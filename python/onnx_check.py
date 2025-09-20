import onnx

model_path = "/home/pi/air_ws/src/Air_SLAM/output/superpoint_v1_sim_int32.onnx"
model = onnx.load(model_path)

onnx.checker.check_model(model)

print("Inputs:")
for input in model.graph.input:
    name = input.name
    shape = [dim.dim_value if (dim.dim_value > 0) else 'dynamic' for dim in input.type.tensor_type.shape.dim]
    dtype = input.type.tensor_type.elem_type
    print(f"  {name} : {shape}, type={dtype}")

print("\nOutputs:")
for output in model.graph.output:
    name = output.name
    shape = [dim.dim_value if (dim.dim_value > 0) else 'dynamic' for dim in output.type.tensor_type.shape.dim]
    dtype = output.type.tensor_type.elem_type
    print(f"  {name} : {shape}, type={dtype}")
