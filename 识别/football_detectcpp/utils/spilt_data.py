import os
import random
import shutil

def split_dataset(
    images_dir="image_files", 
    labels_dir="label_files",
    images_train_dir="images_train",
    labels_train_dir="labels_train",
    images_test_dir="images_test",
    labels_test_dir="labels_test",
    train_ratio=0.8
):
    """
    从指定的图像文件夹和标签文件夹读取数据，然后按 train_ratio 拆分到训练/测试集。
    
    Parameters
    ----------
    images_dir : str
        包含图像文件的目录
    labels_dir : str
        包含标签文件的目录
    images_train_dir : str
        训练集图像输出目录
    labels_train_dir : str
        训练集标签输出目录
    images_test_dir : str
        测试集图像输出目录
    labels_test_dir : str
        测试集标签输出目录
    train_ratio : float
        训练集所占的比例，例如 0.8 表示 80% 的数据用于训练，剩余 20% 用于测试
    """
    # 1. 获取所有图像文件名（只挑选常见后缀）
    valid_image_exts = (".jpg", ".jpeg", ".png", ".bmp")
    image_files = [
        f for f in os.listdir(images_dir)
        if os.path.splitext(f.lower())[1] in valid_image_exts
    ]
    
    # 2. 为防止顺序偏差，先乱序
    random.shuffle(image_files)
    
    # 3. 准备数据对 (image_path, label_path)
    data_pairs = []
    for img_name in image_files:
        # 去除后缀，组合出txt文件名
        base_name = os.path.splitext(img_name)[0]
        txt_name = base_name + ".txt"
        
        img_full_path = os.path.join(images_dir, img_name)
        txt_full_path = os.path.join(labels_dir, txt_name)
        
        # 如果标签文件存在，则将二者一起加入 data_pairs
        if os.path.exists(txt_full_path):
            data_pairs.append((img_full_path, txt_full_path))
        else:
            print(f"[Warning] 标签文件不存在，跳过: {txt_full_path}")
    
    # 4. 计算训练集数量
    train_size = int(len(data_pairs) * train_ratio)
    
    # 5. 拆分数据
    train_pairs = data_pairs[:train_size]
    test_pairs = data_pairs[train_size:]
    
    print(f"数据总数: {len(data_pairs)}")
    print(f"训练集: {len(train_pairs)}")
    print(f"测试集: {len(test_pairs)}")
    
    # 6. 创建输出目录
    os.makedirs(images_train_dir, exist_ok=True)
    os.makedirs(labels_train_dir, exist_ok=True)
    os.makedirs(images_test_dir, exist_ok=True)
    os.makedirs(labels_test_dir, exist_ok=True)
    
    # 7. 拷贝到训练集目录
    for img_path, label_path in train_pairs:
        img_name = os.path.basename(img_path)
        label_name = os.path.basename(label_path)
        shutil.copy2(img_path, os.path.join(images_train_dir, img_name))
        shutil.copy2(label_path, os.path.join(labels_train_dir, label_name))
    
    # 8. 拷贝到测试集目录
    for img_path, label_path in test_pairs:
        img_name = os.path.basename(img_path)
        label_name = os.path.basename(label_path)
        shutil.copy2(img_path, os.path.join(images_test_dir, img_name))
        shutil.copy2(label_path, os.path.join(labels_test_dir, label_name))
    
    print("数据集拆分完成！")


if __name__ == "__main__":
    # 调用split_dataset函数
    split_dataset(
        images_dir="image_files", 
        labels_dir="label_files",
        images_train_dir="images_train",
        labels_train_dir="labels_train",
        images_test_dir="images_test",
        labels_test_dir="labels_test",
        train_ratio=0.8  # 80% 用于训练, 20% 用于测试
    )
