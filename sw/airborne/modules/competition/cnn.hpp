#include <iostream>

#include <vector>
#include <onnxruntime_cxx_api.h>

#define MODEL_PATH "trained.onnx"
#define INPUT_TENSOR_SIZE 4096

class CNN
{
public:
    Ort::Env env;
    Ort::Session *session;
    int64_t *shape;
    const Ort::MemoryInfo memory_info;
    const std::vector<const char *> input_node_names = {"input"};
    const std::vector<const char *> output_node_names = {"output"};

    CNN() : env(Ort::Env(ORT_LOGGING_LEVEL_WARNING, "test")),
            memory_info(Ort::MemoryInfo::CreateCpu(OrtArenaAllocator, OrtMemTypeDefault)),
            shape(new int64_t[4]())
    {
        Ort::SessionOptions session_options;
        session_options.SetIntraOpNumThreads(1);
        session_options.SetGraphOptimizationLevel(GraphOptimizationLevel::ORT_ENABLE_EXTENDED);
        session = new Ort::Session(env, MODEL_PATH, session_options);

        shape[0] = (int64_t)1;
        shape[1] = (int64_t)1;
        shape[2] = (int64_t)64;
        shape[3] = (int64_t)64;
    }

    float* run(float* input)
    {
        // create input tensor object from data values

        // const OrtMemoryInfo* info, T* p_data, size_t p_data_element_count, const int64_t* shape, size_t shape_len
        Ort::Value input_tensor = Ort::Value::CreateTensor<float>(this->memory_info, input, INPUT_TENSOR_SIZE, this->shape, 4);

        // score model & input tensor, get back output tensor
        auto output_tensors = this->session->Run(Ort::RunOptions{nullptr}, input_node_names.data(), &input_tensor, 1, output_node_names.data(), 1);

        return output_tensors.front().GetTensorMutableData<float>();
    }
};