#!/usr/bin/env python3
# robot_lib/parameters.py
import json
import os

def load_parameters(filepath):
    """
    Đọc file JSON chứa các tham số và trả về một dictionary.
    """
    print(f"--- BƯỚC 1: ĐANG NẠP THAM SỐ TỪ '{os.path.basename(filepath)}' ---")
    try:
        with open(filepath, 'r') as f:
            params = json.load(f)
        print(">>> Nạp tham số thành công.")
        return params
    except FileNotFoundError:
        print(f"LỖI: Không tìm thấy file tham số tại '{filepath}'")
        return None
    except json.JSONDecodeError:
        print(f"LỖI: Không thể giải mã JSON từ '{filepath}'. Hãy kiểm tra cú pháp.")
        return None