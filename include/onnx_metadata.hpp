#pragma once

// ONNX metadata parser for mjlab-exported policies.
//
// The training repo (mjlab) writes deployment-relevant parameters into the
// ONNX model's ``metadata_props``.  This header reads those props back via
// the onnxruntime C++ API (Ort::ModelMetadata::GetCustomMetadataMap*) and
// exposes them as a structured OnnxMetadata so the deployment controller can
// auto-populate its config without hardcoding values.
//
// Only the current mjlab exporter layout is understood:
//
//   joint_names / joint_stiffness / joint_damping / default_joint_pos /
//   action_scale
//   onnx_input_names / onnx_input_shapes / onnx_input_groups
//   onnx_output_names / onnx_output_shapes
//   obs_group.<input_name>.history_length
//   obs_group.<input_name>.flatten_history_dim
//   obs_group.<input_name>.term_names
//   obs_group.<input_name>.term_history_lengths
//   obs_group.<input_name>.term_clips
//   obs_group.<input_name>.term_scales
//
// The legacy layout (``observation_names``, ``obs_scale.<term>``,
// ``observation_terms_*``, ``actor_obs_dim`` ...) is intentionally NOT
// supported: models exported with the old schema are rejected at load time.
//
// Serialization format (mirror of mjlab's exporter_utils):
//   - lists are joined with ',' (outer delimiter)
//   - nested sequences (e.g. clip ranges, per-dim scales, shapes) use ';'
//   - floats are formatted with 3 decimals; -inf/inf pass through
//   - non-numeric items (joint/obs names) are kept as plain strings

#include <onnxruntime_cxx_api.h>

#include <algorithm>
#include <cctype>
#include <cmath>
#include <cstddef>
#include <cstdlib>
#include <iostream>
#include <limits>
#include <sstream>
#include <string>
#include <unordered_map>
#include <utility>
#include <vector>

namespace booster_deploy
{

struct ObservationTermCfg
{
    std::string name;
    std::vector<float> scale;      // per-dim scale (default {1.0})
    float clip_min = -std::numeric_limits<float>::infinity();
    float clip_max = std::numeric_limits<float>::infinity();
    int history_length = 0;
};

struct OnnxMetadata
{
    std::string run_path;
    std::string experiment_name;
    std::string run_name;
    std::string git_user_name;

    std::vector<std::string> joint_names;
    std::vector<float> joint_stiffness;
    std::vector<float> joint_damping;
    std::vector<float> default_joint_pos;

    std::vector<std::string> observation_names;
    std::unordered_map<std::string, ObservationTermCfg> observation_terms;

    std::vector<float> action_scale;   // per-joint (scalar expanded)

    std::string observation_group;     // e.g. "actor"
    std::string input_name;            // policy input tensor name
    std::string output_name;           // policy output tensor name
    int actor_obs_dim = 0;
    int actor_history_length = 0;
    bool actor_flatten_history_dim = true;
    int action_dim = 0;

    std::string anchor_body_name;
    std::vector<std::string> body_names;

    // -- term lookups -----------------------------------------------------

    const ObservationTermCfg *term(const std::string &name) const
    {
        auto it = observation_terms.find(name);
        if (it == observation_terms.end())
        {
            std::cerr << "[onnx_metadata] observation term '" << name
                      << "' not found in ONNX metadata; available:";
            for (const auto &kv : observation_terms)
            {
                std::cerr << " " << kv.first;
            }
            std::cerr << std::endl;
            std::abort();
        }
        return &it->second;
    }

    int num_single_obs() const
    {
        return actor_obs_dim / std::max(actor_history_length, 1);
    }

    int num_joints() const { return static_cast<int>(joint_names.size()); }

    void print_summary() const
    {
        std::cout << "ONNX policy metadata:" << std::endl;
        std::cout << "  experiment_name    : " << experiment_name << std::endl;
        std::cout << "  run_name           : " << run_name << std::endl;
        std::cout << "  observation_group  : " << observation_group << std::endl;
        std::cout << "  input / output     : " << input_name << " -> " << output_name << std::endl;
        std::cout << "  actor_obs_dim      : " << actor_obs_dim << std::endl;
        std::cout << "  num_single_obs     : " << num_single_obs() << std::endl;
        std::cout << "  actor_history_len  : " << actor_history_length << std::endl;
        std::cout << "  flatten_history    : " << (actor_flatten_history_dim ? "true" : "false") << std::endl;
        std::cout << "  action_dim         : " << action_dim << std::endl;
        std::cout << "  joints (name / action_scale / default_joint_pos):" << std::endl;
        int num = num_joints();
        for (int i = 0; i < num; ++i)
        {
            float scale = i < static_cast<int>(action_scale.size()) ? action_scale[i] : 0.f;
            float pos = i < static_cast<int>(default_joint_pos.size()) ? default_joint_pos[i] : 0.f;
            std::printf("    %2d. %-24s scale=%8.4f  default=%8.4f\n",
                        i, joint_names[i].c_str(), scale, pos);
        }
        std::cout << "  observation terms:" << std::endl;
        for (const auto &name : observation_names)
        {
            const auto *cfg = term(name);
            std::cout << "    - " << name << ": scale=";
            if (cfg->scale.size() == 1) std::cout << cfg->scale[0];
            else
            {
                std::cout << "[";
                for (size_t i = 0; i < cfg->scale.size(); ++i)
                    std::cout << cfg->scale[i] << (i + 1 < cfg->scale.size() ? ", " : "");
                std::cout << "]";
            }
            std::cout << ", clip=(" << cfg->clip_min << ", " << cfg->clip_max
                      << "), history=" << cfg->history_length << std::endl;
        }
    }
};

// ---------------------------------------------------------------------------
// raw prop parsing (CSV: outer ',', inner ';')
// ---------------------------------------------------------------------------

namespace detail
{

inline std::vector<std::string> split(const std::string &s, char delim)
{
    std::vector<std::string> out;
    std::string cur;
    std::istringstream ss(s);
    while (std::getline(ss, cur, delim))
    {
        out.push_back(cur);
    }
    return out;
}

inline float parse_scalar(const std::string &tok)
{
    if (tok == "-inf") return -std::numeric_limits<float>::infinity();
    if (tok == "inf" || tok == "+inf") return std::numeric_limits<float>::infinity();
    return std::strtof(tok.c_str(), nullptr);
}

// one outer-CSV entry -> scalar or nested list
inline std::vector<float> parse_entry(const std::string &entry)
{
    std::vector<float> out;
    if (entry.find(';') != std::string::npos)
    {
        for (const auto &t : split(entry, ';'))
        {
            out.push_back(parse_scalar(t));
        }
    }
    else
    {
        out.push_back(parse_scalar(entry));
    }
    return out;
}

// full raw value -> vector of entries (each entry a scalar or list)
inline std::vector<std::vector<float>> parse_raw_numeric(const std::string &raw)
{
    std::vector<std::vector<float>> out;
    if (raw.empty()) return out;
    for (const auto &entry : split(raw, ','))
    {
        out.push_back(parse_entry(entry));
    }
    return out;
}

inline std::vector<std::string> parse_raw_strings(const std::string &raw)
{
    std::vector<std::string> out;
    if (raw.empty()) return out;
    for (const auto &entry : split(raw, ','))
    {
        out.push_back(entry);
    }
    return out;
}

inline bool parse_bool(const std::string &raw)
{
    std::string v = raw;
    std::transform(v.begin(), v.end(), v.begin(),
                   [](unsigned char c) { return std::tolower(c); });
    return v == "true" || v == "1";
}

inline std::vector<float> as_float_list(const std::vector<std::vector<float>> &entries)
{
    std::vector<float> out;
    for (const auto &e : entries)
    {
        out.push_back(e.empty() ? 0.f : e[0]);
    }
    return out;
}

inline int entry_to_int(const std::vector<float> &entry)
{
    return entry.empty() ? 0 : static_cast<int>(std::lround(entry[0]));
}

// [batch, d1, d2, ...] -> d1*d2*...  (a bare [d1] shape is also accepted)
inline int shape_entry_size(const std::vector<float> &shape)
{
    if (shape.empty()) return 0;
    if (shape.size() == 1) return static_cast<int>(std::lround(shape[0]));
    int size = 1;
    for (size_t i = 1; i < shape.size(); ++i)
    {
        size *= static_cast<int>(std::lround(shape[i]));
    }
    return size;
}

} // namespace detail

// ---------------------------------------------------------------------------
// public API
// ---------------------------------------------------------------------------

// Extract metadata_props from an onnxruntime session and parse them.
inline OnnxMetadata load_onnx_metadata(Ort::Session &session)
{
    Ort::AllocatorWithDefaultOptions allocator;
    Ort::ModelMetadata meta = session.GetModelMetadata();

    std::unordered_map<std::string, std::string> props;
    auto keys = meta.GetCustomMetadataMapKeysAllocated(allocator);
    for (const auto &key : keys)
    {
        auto value = meta.LookupCustomMetadataMapAllocated(key.get(), allocator);
        if (value) props[key.get()] = value.get();
    }

    auto fail = [](const std::string &msg) {
        std::cerr << "[onnx_metadata] " << msg << std::endl;
        std::abort();
    };

    auto get = [&props](const std::string &k) -> const std::string & {
        static const std::string empty;
        auto it = props.find(k);
        return it == props.end() ? empty : it->second;
    };
    auto require = [&props, &fail](const std::string &k) -> const std::string & {
        auto it = props.find(k);
        if (it == props.end() || it->second.empty())
        {
            fail("missing required metadata key '" + k +
                 "'. The model was not exported with the current mjlab "
                 "exporter (obs_group/onnx_* format); re-export it or use a "
                 "compatible model.");
        }
        return it->second;
    };

    OnnxMetadata m;
    m.run_path = get("run_path");
    m.experiment_name = get("experiment_name");
    m.run_name = get("run_name");
    m.git_user_name = get("git_user_name");

    m.joint_names = detail::parse_raw_strings(require("joint_names"));
    if (m.joint_names.empty()) fail("'joint_names' is empty");
    m.joint_stiffness = detail::as_float_list(detail::parse_raw_numeric(get("joint_stiffness")));
    m.joint_damping = detail::as_float_list(detail::parse_raw_numeric(get("joint_damping")));
    m.default_joint_pos = detail::as_float_list(detail::parse_raw_numeric(get("default_joint_pos")));

    // -- onnx inputs (select the actor group) -----------------------------
    auto input_names = detail::parse_raw_strings(require("onnx_input_names"));
    auto input_shapes = detail::parse_raw_numeric(require("onnx_input_shapes"));
    auto input_groups = detail::parse_raw_strings(require("onnx_input_groups"));
    if (input_names.size() != input_shapes.size() || input_names.size() != input_groups.size())
    {
        fail("onnx_input_names/onnx_input_shapes/onnx_input_groups lengths differ");
    }
    size_t actor_idx = input_groups.size();
    for (size_t i = 0; i < input_groups.size(); ++i)
    {
        if (input_groups[i] == "actor")
        {
            actor_idx = i;
            break;
        }
    }
    if (actor_idx == input_groups.size())
    {
        fail("no 'actor' entry in onnx_input_groups");
    }
    m.observation_group = input_groups[actor_idx];
    m.input_name = input_names[actor_idx];
    m.actor_obs_dim = detail::shape_entry_size(input_shapes[actor_idx]);
    if (m.actor_obs_dim <= 0)
    {
        fail("could not determine actor obs dimension from onnx_input_shapes");
    }

    // -- onnx outputs -----------------------------------------------------
    auto output_names = detail::parse_raw_strings(require("onnx_output_names"));
    auto output_shapes = detail::parse_raw_numeric(require("onnx_output_shapes"));
    if (output_names.empty() || output_names.size() != output_shapes.size())
    {
        fail("onnx_output_names/onnx_output_shapes are empty or have different lengths");
    }
    m.output_name = output_names.front();
    m.action_dim = detail::shape_entry_size(output_shapes.front());

    // -- role metadata ----------------------------------------------------
    // action_scale: scalar -> per-joint, or explicit list
    auto scale_entries = detail::parse_raw_numeric(get("action_scale"));
    if (scale_entries.size() <= 1)
    {
        float s = scale_entries.empty() ? 1.0f : (scale_entries[0].empty() ? 1.0f : scale_entries[0][0]);
        m.action_scale.assign(std::max(m.num_joints(), 1), s);
    }
    else
    {
        m.action_scale = detail::as_float_list(scale_entries);
    }
    if (static_cast<int>(m.action_scale.size()) != m.num_joints())
    {
        fail("action_scale size (" + std::to_string(m.action_scale.size()) +
             ") != number of joints (" + std::to_string(m.num_joints()) + ")");
    }

    m.anchor_body_name = get("anchor_body_name");
    m.body_names = detail::parse_raw_strings(get("body_names"));

    // -- observation group config ----------------------------------------
    const std::string prefix = "obs_group." + m.input_name + ".";
    auto require_obs = [&prefix, &require](const std::string &suffix) -> const std::string & {
        return require(prefix + suffix);
    };

    m.actor_history_length = detail::entry_to_int(
        detail::parse_entry(require_obs("history_length")));
    m.actor_flatten_history_dim = detail::parse_bool(require_obs("flatten_history_dim"));

    m.observation_names = detail::parse_raw_strings(require_obs("term_names"));
    if (m.observation_names.empty()) fail("'" + prefix + "term_names' is empty");

    auto term_scales = detail::parse_raw_numeric(require_obs("term_scales"));
    auto term_clips = detail::parse_raw_numeric(require_obs("term_clips"));
    auto term_history = detail::parse_raw_numeric(require_obs("term_history_lengths"));
    if (term_scales.size() != m.observation_names.size() ||
        term_clips.size() != m.observation_names.size() ||
        term_history.size() != m.observation_names.size())
    {
        fail("obs_group term arrays do not match term_names length (" +
             std::to_string(m.observation_names.size()) + ")");
    }

    for (size_t i = 0; i < m.observation_names.size(); ++i)
    {
        ObservationTermCfg cfg;
        cfg.name = m.observation_names[i];

        if (!term_scales[i].empty()) cfg.scale = term_scales[i];
        else cfg.scale = {1.0f};

        if (term_clips[i].size() >= 2)
        {
            cfg.clip_min = term_clips[i][0];
            cfg.clip_max = term_clips[i][1];
        }
        cfg.history_length = detail::entry_to_int(term_history[i]);
        m.observation_terms[cfg.name] = std::move(cfg);
    }

    if (m.actor_history_length <= 0)
    {
        fail("obs_group '" + m.input_name + "' has non-positive history_length");
    }
    if (m.action_dim > 0 && m.action_dim != m.num_joints())
    {
        fail("model action dim (" + std::to_string(m.action_dim) +
             ") != number of joints (" + std::to_string(m.num_joints()) + ")");
    }

    return m;
}

} // namespace booster_deploy
