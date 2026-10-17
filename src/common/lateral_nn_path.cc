#include "common/lateral_nn_path.h"

#include "common/json_value.h"
#include "common/utils_file.h"
#include "common/utils_process.h"

bool valid_lateral_nn_name(const std::string &name) {
    if (name.empty() || name.size() > 64 || name[0] == '.') return false;
    for (char c : name) {
        const bool ok = (c >= 'a' && c <= 'z') || (c >= 'A' && c <= 'Z') || (c >= '0' && c <= '9') || c == '_' ||
                        c == '-' || c == '.';
        if (!ok) return false;
    }
    return true;
}

std::string lateral_nn_selection_path(const std::string &params_directory) {
    return params_directory + "/" + kLateralNnSelectionFile;
}

LateralNnChoice lateral_nn_choice_from(const std::string &selection_text, const std::string &model_dir) {
    LateralNnChoice choice;
    if (selection_text.empty()) {
        choice.error = "no model chosen (no " + std::string(kLateralNnSelectionFile) + ")";
        return choice;
    }
    JsonValue root;
    std::string error;
    if (!JsonValue::parse(selection_text, &root, &error)) {
        choice.error = "selection: " + error;
        return choice;
    }
    const JsonValue *model = root.find("model");
    if (!model || !model->is_string()) {
        choice.error = "selection: no \"model\" name";
        return choice;
    }
    choice.name = model->string();
    if (!valid_lateral_nn_name(choice.name)) {
        choice.error = "selection: bad model name \"" + choice.name + "\"";
        return choice;
    }
    choice.path = model_dir + "/" + choice.name + ".json";
    return choice;
}

LateralNnChoice lateral_nn_choice(const std::string &params_directory) {
    const std::string fixed = env_string("EDGEPILOT_LATERAL_NN_MODEL");
    if (!fixed.empty()) return LateralNnChoice{fixed, fixed, ""};
    return lateral_nn_choice_from(read_text_file(lateral_nn_selection_path(params_directory)), kLateralNnModelDir);
}

LateralNnChoice lateral_nn_choice() { return lateral_nn_choice(params_dir()); }
