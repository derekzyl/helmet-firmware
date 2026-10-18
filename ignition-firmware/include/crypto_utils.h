#pragma once

#include <string>
#include <vector>
#include <sstream>
#include <iomanip>
#include <cstdint>
#include <cstring>

// ==============================================================================
// EMBEDDED SHA-256 & HMAC-SHA256 UTILITIES (IGNITION CONTROLLER)
// Pure C++ portable implementation for both Native Testing and ESP32/FreeRTOS
// ==============================================================================

namespace crypto {

namespace detail {

inline uint32_t rotr(uint32_t n, uint32_t c) {
    return (n >> c) | (n << (32 - c));
}

inline uint32_t ch(uint32_t x, uint32_t y, uint32_t z) {
    return (x & y) ^ (~x & z);
}

inline uint32_t maj(uint32_t x, uint32_t y, uint32_t z) {
    return (x & y) ^ (x & z) ^ (y & z);
}

inline uint32_t sig0(uint32_t x) {
    return rotr(x, 2) ^ rotr(x, 13) ^ rotr(x, 22);
}

inline uint32_t sig1(uint32_t x) {
    return rotr(x, 6) ^ rotr(x, 11) ^ rotr(x, 25);
}

inline uint32_t theta0(uint32_t x) {
    return rotr(x, 7) ^ rotr(x, 18) ^ (x >> 3);
}

inline uint32_t theta1(uint32_t x) {
    return rotr(x, 17) ^ rotr(x, 19) ^ (x >> 10);
}

static const uint32_t K[64] = {
    0x428a2f98, 0x71374491, 0xb5c0fbcf, 0xe9b5dba5, 0x3956c25b, 0x59f111f1, 0x923f82a4, 0xab1c5ed5,
    0xd807aa98, 0x12835b01, 0x243185be, 0x550c7dc3, 0x72be5d74, 0x80deb1fe, 0x9bdc06a7, 0xc19bf174,
    0xe49b69c1, 0xefbe4786, 0x0fc19dc6, 0x240ca1cc, 0x2de92c6f, 0x4a7484aa, 0x5cb0a9dc, 0x76f988da,
    0x983e5152, 0xa831c66d, 0xb00327c8, 0xbf597fc7, 0xc6e00bf3, 0xd5a79147, 0x06ca6351, 0x14292967,
    0x27b70a85, 0x2e1b2138, 0x4d2c6dfc, 0x53380d13, 0x650a7354, 0x766a0abb, 0x81c2c92e, 0x92722c85,
    0xa2bfe8a1, 0xa81a664b, 0xc24b8b70, 0xc76c51a3, 0xd192e819, 0xd6990624, 0xf40e3585, 0x106aa070,
    0x19a4c116, 0x1e376c08, 0x2748774c, 0x34b0bcb5, 0x391c0cb3, 0x4ed8aa4a, 0x5b9cca4f, 0x682e6ff3,
    0x748f82ee, 0x78a5636f, 0x84c87814, 0x8cc70208, 0x90befffa, 0xa4506ceb, 0xbef9a3f7, 0xc67178f2
};

} // namespace detail

class SHA256 {
public:
    SHA256() { reset(); }

    void reset() {
        m_len = 0;
        m_tot_len = 0;
        m_h[0] = 0x6a09e667;
        m_h[1] = 0xbb67ae85;
        m_h[2] = 0x3c6ef372;
        m_h[3] = 0xa54ff53a;
        m_h[4] = 0x510e527f;
        m_h[5] = 0x9b05688c;
        m_h[6] = 0x1f83d9ab;
        m_h[7] = 0x5be0cd19;
    }

    void update(const uint8_t* message, size_t len) {
        for (size_t i = 0; i < len; ++i) {
            m_block[m_len++] = message[i];
            if (m_len == 64) {
                transform();
                m_tot_len += 512;
                m_len = 0;
            }
        }
    }

    void update(const std::string& str) {
        update(reinterpret_cast<const uint8_t*>(str.data()), str.size());
    }

    std::vector<uint8_t> digest() {
        uint8_t hash[32];
        final(hash);
        return std::vector<uint8_t>(hash, hash + 32);
    }

    std::string digest_hex() {
        uint8_t hash[32];
        final(hash);
        std::ostringstream ss;
        for (int i = 0; i < 32; ++i) {
            ss << std::hex << std::setw(2) << std::setfill('0') << static_cast<int>(hash[i]);
        }
        return ss.str();
    }

    void final(uint8_t* digest) {
        uint32_t block_nb = (1 + ((64 - 9) < (m_len % 64)));
        size_t len_b = (m_tot_len + (m_len << 3));
        size_t pm_len = block_nb << 6;

        memset(m_block + m_len, 0, pm_len - m_len);
        m_block[m_len] = 0x80;

        for (int i = 0; i < 8; ++i) {
            m_block[pm_len - 1 - i] = static_cast<uint8_t>((len_b >> (i * 8)) & 0xFF);
        }

        transform();
        if (block_nb == 2) {
            transform();
        }

        for (int i = 0; i < 8; ++i) {
            digest[i * 4 + 0] = static_cast<uint8_t>((m_h[i] >> 24) & 0xFF);
            digest[i * 4 + 1] = static_cast<uint8_t>((m_h[i] >> 16) & 0xFF);
            digest[i * 4 + 2] = static_cast<uint8_t>((m_h[i] >> 8) & 0xFF);
            digest[i * 4 + 3] = static_cast<uint8_t>((m_h[i]) & 0xFF);
        }
    }

private:
    void transform() {
        uint32_t w[64];
        for (int i = 0; i < 16; ++i) {
            w[i] = (static_cast<uint32_t>(m_block[i * 4 + 0]) << 24) |
                   (static_cast<uint32_t>(m_block[i * 4 + 1]) << 16) |
                   (static_cast<uint32_t>(m_block[i * 4 + 2]) << 8)  |
                   (static_cast<uint32_t>(m_block[i * 4 + 3]));
        }
        for (int i = 16; i < 64; ++i) {
            w[i] = detail::theta1(w[i - 2]) + w[i - 7] + detail::theta0(w[i - 15]) + w[i - 16];
        }

        uint32_t a = m_h[0], b = m_h[1], c = m_h[2], d = m_h[3];
        uint32_t e = m_h[4], f = m_h[5], g = m_h[6], h = m_h[7];

        for (int i = 0; i < 64; ++i) {
            uint32_t t1 = h + detail::sig1(e) + detail::ch(e, f, g) + detail::K[i] + w[i];
            uint32_t t2 = detail::sig0(a) + detail::maj(a, b, c);
            h = g;
            g = f;
            f = e;
            e = d + t1;
            d = c;
            c = b;
            b = a;
            a = t1 + t2;
        }

        m_h[0] += a;
        m_h[1] += b;
        m_h[2] += c;
        m_h[3] += d;
        m_h[4] += e;
        m_h[5] += f;
        m_h[6] += g;
        m_h[7] += h;
    }

    uint8_t m_block[128];
    size_t m_len;
    size_t m_tot_len;
    uint32_t m_h[8];
};

inline std::vector<uint8_t> hmac_sha256(const std::vector<uint8_t>& key, const uint8_t* data, size_t len) {
    uint8_t k_pad[64];
    memset(k_pad, 0, sizeof(k_pad));

    if (key.size() > 64) {
        SHA256 k_sha;
        k_sha.update(key.data(), key.size());
        auto k_hash = k_sha.digest();
        memcpy(k_pad, k_hash.data(), k_hash.size());
    } else {
        memcpy(k_pad, key.data(), key.size());
    }

    uint8_t ipad[64];
    uint8_t opad[64];
    for (int i = 0; i < 64; ++i) {
        ipad[i] = k_pad[i] ^ 0x36;
        opad[i] = k_pad[i] ^ 0x5C;
    }

    SHA256 inner;
    inner.update(ipad, 64);
    inner.update(data, len);
    auto inner_digest = inner.digest();

    SHA256 outer;
    outer.update(opad, 64);
    outer.update(inner_digest.data(), inner_digest.size());
    return outer.digest();
}

inline std::string hmac_sha256_hex(const std::string& key_str, const std::string& msg) {
    std::vector<uint8_t> key(key_str.begin(), key_str.end());
    auto digest = hmac_sha256(key, reinterpret_cast<const uint8_t*>(msg.data()), msg.size());

    std::ostringstream ss;
    for (uint8_t b : digest) {
        ss << std::hex << std::setw(2) << std::setfill('0') << static_cast<int>(b);
    }
    return ss.str();
}

inline bool constant_time_compare(const std::string& a, const std::string& b) {
    if (a.size() != b.size()) return false;
    uint8_t diff = 0;
    for (size_t i = 0; i < a.size(); ++i) {
        diff |= static_cast<uint8_t>(a[i] ^ b[i]);
    }
    return diff == 0;
}

inline bool verify_hmac(const std::string& key, const std::string& msg, const std::string& expected_hmac) {
    std::string calculated = hmac_sha256_hex(key, msg);
    return constant_time_compare(calculated, expected_hmac);
}

} // namespace crypto
