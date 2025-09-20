import onnx
import onnx_graphsurgeon as gs
import numpy as np
from onnx import TensorProto

in_model = "/home/pi/air_ws/src/Air_SLAM/output/superpoint_v1_sim_int32.onnx"
out_model = "/home/pi/air_ws/src/Air_SLAM/output/superpoint_v1_fixed.onnx"

graph = gs.import_onnx(onnx.load(in_model))

def cast_input(var):
    if var.dtype == np.int64 or var.dtype is None:
        cast_out = gs.Variable(
            name=(var.name or "anon") + "_cast",
            dtype=np.int32,
            shape=var.shape
        )
        cast_node = gs.Node(
            op="Cast",
            name=(var.name or "anon") + "_cast_node",
            inputs=[var],
            outputs=[cast_out],
            attrs={"to": TensorProto.INT32}
        )
        graph.nodes.append(cast_node)
        return cast_out
    return var

for node in graph.nodes:
    if node.name in ["Concat_43", "Mul_47", "Mul_49", "Concat_82", "Concat_53"]:
        node.inputs = [cast_input(inp) for inp in node.inputs]

graph.cleanup().toposort()
onnx.save(gs.export_onnx(graph), out_model)
print(f"[INFO] Fixed model saved -> {out_model}")




