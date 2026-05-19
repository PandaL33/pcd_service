def convert_coordinates(input_coords):
    """
    将输入的坐标对象数组转换为目标格式
    
    参数:
    input_coords: 包含x,y坐标的对象列表，例如 [{"x": 1.0, "y": 2.0}, {"x": 3.0, "y": 4.0}]
    
    返回:
    嵌套的坐标对列表，例如 [[[x1,y1], [x2,y2], ...]]
    """
    converted = []
    for coord in input_coords:
        converted.append([coord["x"], coord["y"]])
    
    # 根据需求，将整个列表包装在一个额外的数组中
    return [converted]


# 使用示例
def example_usage():
    """演示如何使用 coordinate_converter 模块"""
    print("=== 坐标转换器使用示例 ===")
    
    # 示例输入数据
    input_data = [
        {"x": 0.05882544629203608, "y": 15.402801517826518},
        {"x": -0.6837954594241182, "y": 10.410918271551408},
        {"x": -0.6399707259451533, "y": -6.579392576454895},
        {"x": 49.37073149113101, "y": -6.577439605943424},
        {"x": 48.888716876682906, "y": 15.428741055700124},
        {"x": 24.527032145423036, "y": 15.512123005695617},
        {"x": 0.05882544629203608, "y": 15.402801517826518}
    ]
    
    print("输入数据:")
    for i, coord in enumerate(input_data):
        print(f"  {i+1}: x={coord['x']}, y={coord['y']}")
    
    # 调用转换函数
    result = convert_coordinates(input_data)
    
    print("\n输出数据:")
    print("  转换后的嵌套坐标对列表:")
    for i, coord_pair in enumerate(result[0]):
        print(f"    [{coord_pair[0]}, {coord_pair[1]}]")
    
    print(f"\n完整输出: {result}")


def usage_as_module():
    """演示如何在其他模块中导入和使用此功能"""
    print("\n=== 如何在其他模块中使用 ===")
    print("# 在其他 Python 文件中可以这样导入和使用:")
    print("from coordinate_converter import convert_coordinates")
    print("")
    print("# 然后使用转换函数:")
    print("input_data = [{\"x\": 1.0, \"y\": 2.0}, {\"x\": 3.0, \"y\": 4.0}]")
    print("result = convert_coordinates(input_data)")
    print("print(result)  # 输出: [[[1.0, 2.0], [3.0, 4.0]]]")


if __name__ == "__main__":
    # 当直接运行此文件时，执行示例
    example_usage()
    # usage_as_module()