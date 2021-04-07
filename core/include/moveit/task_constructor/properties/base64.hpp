/**
 * @file base64.h
 * @author Basit Ayantunde (rlamarrr@gmail.com)
 * @brief
 * @version 0.1
 * @date 2019-09-08
 *
 * @copyright Copyright (c) 2019
 *
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *    http://www.apache.org/licenses/LICENSE-2.0

 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 *
 */

#ifndef LAMAR_BASE64_H
#define LAMAR_BASE64_H
#include <cstring>
#include <string>

namespace base64 {

template <typename T = char>
struct Base64Chars
{
	static constexpr T data[]{ "ABCDEFGHIJKLMNOPQRSTUVWXYZ"
		                        "abcdefghijklmnopqrstuvwxyz"
		                        "0123456789+/" };
};

template <typename T>
constexpr T Base64Chars<T>::data[];

template <typename T = char>
inline bool is_base64(T c) {
	return (isalnum(c) || (c == '+') || (c == '/'));
}

template <typename T = char>
std::basic_string<T> encode(const T* bytes_to_encode, uint32_t in_len) {
	std::basic_string<T> ret;
	int i = 0;
	int j = 0;
	T char_array_3[3];
	T char_array_4[4];

	while (in_len--) {
		char_array_3[i++] = *(bytes_to_encode++);
		if (i == 3) {
			char_array_4[0] = (char_array_3[0] & 0xfc) >> 2;
			char_array_4[1] = ((char_array_3[0] & 0x03) << 4) + ((char_array_3[1] & 0xf0) >> 4);
			char_array_4[2] = ((char_array_3[1] & 0x0f) << 2) + ((char_array_3[2] & 0xc0) >> 6);
			char_array_4[3] = char_array_3[2] & 0x3f;

			for (i = 0; (i < 4); i++)
				ret += Base64Chars<T>::data[char_array_4[i]];
			i = 0;
		}
	}

	if (i) {
		for (j = i; j < 3; j++)
			char_array_3[j] = '\0';

		char_array_4[0] = (char_array_3[0] & 0xfc) >> 2;
		char_array_4[1] = ((char_array_3[0] & 0x03) << 4) + ((char_array_3[1] & 0xf0) >> 4);
		char_array_4[2] = ((char_array_3[1] & 0x0f) << 2) + ((char_array_3[2] & 0xc0) >> 6);
		char_array_4[3] = char_array_3[2] & 0x3f;

		for (j = 0; (j < i + 1); j++)
			ret += Base64Chars<T>::data[char_array_4[j]];

		while ((i++ < 3))
			ret += '=';
	}

	return std::move(ret);
}

template <typename T = char>
std::basic_string<T> decode(const T* encoded_string, int64_t in_len) {
	int i = 0;
	int j = 0;
	int in_ = 0;
	T char_array_4[4], char_array_3[3];
	std::basic_string<T> ret;

	while (in_len-- && (encoded_string[in_] != '=') && is_base64(encoded_string[in_])) {
		char_array_4[i++] = encoded_string[in_];
		in_++;
		if (i == 4) {
			for (i = 0; i < 4; i++)
				char_array_4[i] = strchr(Base64Chars<T>::data, char_array_4[i]) - Base64Chars<T>::data;

			char_array_3[0] = (char_array_4[0] << 2) + ((char_array_4[1] & 0x30) >> 4);
			char_array_3[1] = ((char_array_4[1] & 0xf) << 4) + ((char_array_4[2] & 0x3c) >> 2);
			char_array_3[2] = ((char_array_4[2] & 0x3) << 6) + char_array_4[3];

			for (i = 0; (i < 3); i++)
				ret += char_array_3[i];
			i = 0;
		}
	}

	if (i) {
		for (j = i; j < 4; j++)
			char_array_4[j] = 0;

		for (j = 0; j < 4; j++)
			char_array_4[j] = strchr(Base64Chars<T>::data, char_array_4[j]) - Base64Chars<T>::data;

		char_array_3[0] = (char_array_4[0] << 2) + ((char_array_4[1] & 0x30) >> 4);
		char_array_3[1] = ((char_array_4[1] & 0xf) << 4) + ((char_array_4[2] & 0x3c) >> 2);
		char_array_3[2] = ((char_array_4[2] & 0x3) << 6) + char_array_4[3];

		for (j = 0; (j < i - 1); j++)
			ret += char_array_3[j];
	}

	return std::move(ret);
}

};  // namespace base64

#endif
