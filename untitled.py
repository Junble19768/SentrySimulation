import numpy as np

def standardize_tuple(input_tuple):
    # 将元组转换为 NumPy 数组
    input_array = np.array(input_tuple, dtype=float)
    
    # 计算均值和标准差
    mean_value = np.mean(input_array)
    std_deviation = np.std(input_array)
    
    # 标准化处理
    standardized_array = (input_array - mean_value) / std_deviation
    
    # 将 NumPy 数组转换回元组
    standardized_tuple = tuple(standardized_array)
    
    return standardized_tuple

# 示例用法
original_tuple = (1.0, 2.0, 3.0, 4.0, 5.0)
standardized_result = standardize_tuple(original_tuple)

print("Original Tuple:", original_tuple)
print("Standardized Tuple:", standardized_result)
