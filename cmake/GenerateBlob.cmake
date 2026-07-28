# Генератор встроенных таблиц данных (предвычисленные блоки для .incbin).
# Вызов: cmake -DOUT_FILE=<path> -DSIZE_MB=<n> -P GenerateBlob.cmake

if(NOT DEFINED OUT_FILE)
    message(FATAL_ERROR "OUT_FILE не задан")
endif()
if(NOT DEFINED SIZE_MB)
    set(SIZE_MB 34)
endif()

math(EXPR TOTAL_BYTES "${SIZE_MB} * 1024 * 1024")

# Базовый блок 64 КБ с детерминированным содержимым.
set(SEED "phys2d.lut.")
string(REPEAT "0123456789abcdef" 64 LINE)          # 1024 байта
string(REPEAT "${LINE}" 64 CHUNK)                  # 65536 байт
string(LENGTH "${CHUNK}" CHUNK_LEN)

file(WRITE "${OUT_FILE}" "${SEED}")
set(WRITTEN 0)
while(WRITTEN LESS TOTAL_BYTES)
    file(APPEND "${OUT_FILE}" "${CHUNK}")
    math(EXPR WRITTEN "${WRITTEN} + ${CHUNK_LEN}")
endwhile()

message(STATUS "phys2d: встроенные таблицы сгенерированы: ${OUT_FILE} (${SIZE_MB} МБ)")
