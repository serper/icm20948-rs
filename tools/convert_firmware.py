# convert_firmware.py
import re
import sys


def convert_c_array_to_rust():
    input_file = "icm20948_img.dmp3a.h"
    output_file = "dmp3_firmware.rs"

    with open(input_file, "r") as f:
        content = f.read()

    # Extraer todos los valores hexadecimales (0xXX) del archivo
    hex_pattern = re.compile(r"0x[0-9a-fA-F]{2}")
    values = hex_pattern.findall(content)

    # Si no se encontraron valores hexadecimales
    if not values:
        print("No se encontraron valores hexadecimales en el archivo")
        return

    # Agrupar en líneas de 8 valores para mejor legibilidad
    rust_lines = []
    for i in range(0, len(values), 8):
        rust_lines.append("    " + ", ".join(values[i : i + 8]) + ",")

    rust_code = "pub const DMP3_FIRMWARE: &[u8] = &[\n"
    rust_code += "\n".join(rust_lines)
    rust_code += "\n];\n"

    with open(output_file, "w") as f:
        f.write(rust_code)

    print(f"Conversión completada. Se han convertido {len(values)} bytes.")


if __name__ == "__main__":
    convert_c_array_to_rust()
