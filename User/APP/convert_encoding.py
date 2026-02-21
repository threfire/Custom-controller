import os
import sys

def convert_file_to_utf8(file_path):
    """娑娴GB2312娑UTF-8"""
    try:
        # 璇GB2312缂娴
        with open(file_path, 'r', encoding='gb2312', errors='ignore') as f:
            content = f.read()
        
        # UTF-8缂
        with open(file_path, 'w', encoding='utf-8') as f:
            f.write(content)
        print(f" 瀹歌舵: {file_path}")
        return True
    except Exception as e:
        print(f" : {file_path} - {e}")
        return False

def convert_all_files(root_folder):
    """瑜版℃"""
    supported_extensions = ['.txt', '.csv', '.html', '.htm', '.xml', 
                           '.json', '.js', '.css', '.py', '.java', 
                           '.cpp', '.c', '.h', '.md', '.ini', '.cfg', 
                           '.conf', '.properties']
    
    converted = 0
    failed = 0
    
    for root, dirs, files in os.walk(root_folder):
        for file in files:
            # 濡娴碘
            if any(file.endswith(ext) for ext in supported_extensions):
                file_path = os.path.join(root, file)
                if convert_file_to_utf8(file_path):
                    converted += 1
                else:
                    failed += 1
    
    print(f"\n")
    print(f": {converted} 娑娴")
    print(f": {failed} 娑娴")
    return converted, failed

if __name__ == "__main__":
    # 宄拌ぐ娴璺恒
    current_folder = os.path.dirname(os.path.abspath(__file__))
    
    print(f"℃璺恒: {current_folder}")
    print("娴琚: .txt, .csv, .html, .xml, .json, .js, .css, .py 缁娴")
    
    confirm = input("绾婵(y/n): ").lower()
    if confirm == 'y':
        convert_all_files(current_folder)
    else:
        print("娴ｅ告濞")
