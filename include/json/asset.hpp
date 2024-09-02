#pragma once

#include <stdint.h>
#include <cstddef>
#include "json.hpp"

using Json = json11::Json;

extern "C" {

typedef struct __attribute__((__packed__)) _asset {
	uint8_t* buf;
	size_t size;
} asset;
}

/**
 * Open an asset file as a json object
 *
 * @param x The asset file, first opened with the macro, ASSET(x)
 * @return A json object from the asset file
 *
 * @code
 * ASSET(FILE_json);
 *
 * json parsed_file = open_asset_as_json(FILE_json);
 * @endcode
 */
Json open_asset_as_json(asset x);

#define ASSET(x)                                                                                                       \
    extern "C" {                                                                                                       \
    extern uint8_t _binary_static_##x##_start[], _binary_static_##x##_size[];                                          \
    static asset x = {_binary_static_##x##_start, (size_t)_binary_static_##x##_size};                                  \
    }
