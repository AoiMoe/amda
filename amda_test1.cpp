//
// Copyright (c)2008,2016 Takuya SHIOZAKI,
// All rights reserved.
//
// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions
// are met:
// 1. Redistributions of source code must retain the above copyright
//    notice, this list of conditions and the following disclaimer.
// 2. Redistributions in binary form must reproduce the above copyright
//    notice, this list of conditions and the following disclaimer in the
//    documentation and/or other materials provided with the distribution.
//
// THIS SOFTWARE IS PROVIDED BY THE AUTHOR AND CONTRIBUTORS ``AS IS'' AND
// ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
// IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
// ARE DISCLAIMED.  IN NO EVENT SHALL THE AUTHOR OR CONTRIBUTORS BE LIABLE
// FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
// DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS
// OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION)
// HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
// LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY
// OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF
// SUCH DAMAGE.
//

#define AMDA_DEBUG
#include <amda.h>
#include <cstdio>

using namespace AMDA;
using namespace std;

#if !defined(TEST_SEPARATED) && !defined(TEST_STRUCTURED) &&                   \
    !defined(TEST_DELTA_CHECK)
#error TEST_* not defined.
#endif

#if defined(TEST_SEPARATED) || defined(TEST_STRUCTURED)
#ifdef TEST_SEPARATED
#define STORAGE Standard::SeparatedStorage
#else
#define STORAGE Standard::StructuredStorage
#endif
using TR = Standard::Traits<char, size_t, unsigned int, STORAGE>;
using DA = DoubleArray<TR>;
template <class E_>
using ScratchSource = AMDA::Standard::StructuredScratchSource<TR, E_>;
using FileDrain = AMDA::Standard::FileDrain<TR>;
using FileSource = AMDA::Standard::FileSource<TR>;
using CharType = char;
#else // TEST_DELTA_CHECK
using TR = DeltaCheck::Traits<size_t, unsigned int>;
using DA = DoubleArray<TR>;
template <class E_>
using ScratchSource = AMDA::DeltaCheck::StructuredScratchSource<TR, E_>;
using FileDrain = AMDA::DeltaCheck::FileDrain<TR>;
using FileSource = AMDA::DeltaCheck::FileSource<TR>;
using CharType = AMDA::U8;
#endif
using ArrayBody = TR::ArrayBody;
using KeyType = const CharType *;
using NodeIDType = TR::NodeIDType;
using SizeType = TR::SizeType;

class CPCallback {
public:
    Status operator()(const DA::Walker &w) const {
        printf("    prefix \"%.*s\" found: %u\n", static_cast<int>(w.depth()),
               w.key(), w.id());
        return S_OK;
    }
};

class Key {
public:
    template <typename Ch_, SizeType l>
    Key(const Ch_ (&k)[l], NodeIDType nid = 0)
        : m_key{reinterpret_cast<KeyType>(k)}, m_keylen{l - 1}, m_nid{nid} {}
    Key() {}
    operator bool() const { return m_key; }
    NodeIDType leaf_id() const { return m_nid; }
    KeyType key() const { return m_key; }
    SizeType key_length() const { return m_keylen; }

private:
    KeyType m_key = nullptr;
    size_t m_keylen = 0;
    NodeIDType m_nid = 0;
};

void test_walker(const DA &da, Key key, Key subkey = Key{}) {
    DA::Walker w(da, key.key(), key.key_length());
    Status rv;

    printf("test_walker(%.*s)\n", static_cast<int>(key.key_length()),
           key.key());
retry:
    do {
        printf("  id=%u, depth=%d\n", w.id(), static_cast<int>(w.depth()));
    } while (!(rv = w(CPCallback())));
    if (rv == S_BREAK)
        printf("  found: ");
    else {
        printf("  not found.\n");
        if (rv == S_NO_ENTRY && subkey) {
            printf("  retry by subkey(%.*s)\n",
                   static_cast<int>(subkey.key_length()), subkey.key());
            w = DA::Walker(w, subkey.key(), subkey.key_length());
            subkey = Key{};
            goto retry;
        }
        printf("  error=%d: ", rv);
    }
    printf("id=%u, depth=%d\n", w.id(), static_cast<int>(w.depth()));
}

template <class Policy_>
void test_common(const char *title, const DA &da, Key key) {
    printf("%s(%.*s)\n", title, static_cast<int>(key.key_length()), key.key());

    da.find<Policy_>(key.key(), key.key_length())
        .apply([](auto w) {
            printf("  found: %.*s\n", static_cast<int>(w.depth()),
                   reinterpret_cast<const char *>(w.key()));
        })
        .failure([](auto rv) {
            if (rv == S_NO_ENTRY)
                printf("  not found\n");
            else if (rv)
                printf("  error=%d\n", rv);
        });
}

void test_exact(const DA &da, Key key) {
    test_common<DA::Walker::ExactPolicy>("test_exact", da, key);
}

void test_most_common(const DA &da, Key key) {
    test_common<DA::Walker::MostCommonPolicy>("test_most_common", da, key);
}

void test_least_common(const DA &da, Key key) {
    test_common<DA::Walker::LeastCommonPolicy>("test_least_common", da, key);
}

#define NUM_OF(a) (sizeof(a) / sizeof(*a))
int main() {
    Key keys[] = {
#ifdef TEST_NULL_STRING
        Key{"", 1},
#endif
        Key{"a", 2}, Key{"aa", 3}, Key{"bb", 4}, Key{"bc", 5}};

    DA::build(ScratchSource<Key>{keys})
        // Failable<DA>
        .apply([](auto da) {
            printf("dump.\n");
            return da.dump(FileDrain("test1.da"));
        })
        // Failable<void>
        .apply([]() {
            printf("restore.\n");

            return DA::build(FileSource("test1.da"));
        })
        // Failable<DA>
        .apply([](auto da) {
            const ArrayBody &ab = da.array_body();
            printf("[0]; base=%u(node)\n", ab.base(0, 0));
            for (size_t i = 1; i < ab.num_entries(); i++) {
                if (ab.is_inuse(i, 0)) {
                    printf("[%d]; check=%u, base=%u(%s)\n", static_cast<int>(i),
                           ab.check(i, 0), ab.base(i, 0),
                           ab.check(i, 0) == i ? "leaf" : "node");
                }
            }

            test_walker(da, Key{"a"});
            test_walker(da, Key{"b"});
            test_walker(da, Key{"aa"});
            test_walker(da, Key{"ab"}, Key{"a"});
            test_walker(da, Key{"bb"});
            test_walker(da, Key{"bc"});
            test_walker(da, Key{"bcc"}, Key{""});
            test_walker(da, Key{"x"});

            test_exact(da, Key{"a"});
            test_exact(da, Key{"aa"});
            test_exact(da, Key{"aaa"});

            test_most_common(da, Key{"a"});
            test_most_common(da, Key{"aa"});
            test_most_common(da, Key{"aaa"});

            test_least_common(da, Key{"a"});
            test_least_common(da, Key{"aa"});
            test_least_common(da, Key{"aaa"});
        })
        // Failable<void>
        .failure([](auto rv) {
            printf("error=%d\n", static_cast<int>(rv));
            exit(EXIT_FAILURE);
        });

    return EXIT_SUCCESS;
}
