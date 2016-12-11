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
#include <iostream>
#include <string>
#include <set>

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
using SortedKeySource = AMDA::Standard::SortedKeySource<TR>;
using ArrayBody = TR::ArrayBody;
using FileDrain = AMDA::Standard::FileDrain<TR>;
using FileSource = AMDA::Standard::FileSource<TR>;
using KeyType = const char *;
#else // TEST_DELTA_CHECK
using TR = DeltaCheck::Traits<size_t, unsigned int>;
using DA = DoubleArray<TR>;
using SortedKeySource = AMDA::DeltaCheck::SortedKeySource<TR>;
using ArrayBody = TR::ArrayBody;
using FileDrain = AMDA::DeltaCheck::FileDrain<TR>;
using FileSource = AMDA::DeltaCheck::FileSource<TR>;
using KeyType = const AMDA::U8 *;
#endif

using KeySet = set<string>;

int main() {
    Status rv;
    KeySet keyset;

    // read keys from stdin - separated by whitespaces.
    while (!cin.eof()) {
        string key;
        cin >> key;
        if (key.size())
            keyset.insert(key);
    }

    // build sorted array of keys.
    KeyType *keys = new KeyType[keyset.size()];
    size_t *keylens = new size_t[keyset.size()];
    int count = 0;
    for (KeySet::const_iterator i = keyset.begin(); i != keyset.end(); ++i) {
        keylens[count] = i->size();
        keys[count] = reinterpret_cast<KeyType>(i->c_str());
        count++;
    }

    // build double array.
    DA da;
    rv = da.build(SortedKeySource(count, keys, keylens));
    if (rv) {
        cerr << "error=" << static_cast<int>(rv) << endl;
        return 1;
    }

    // test dump/restore to/from file.
    cout << "dump." << endl;
    rv = da.dump(FileDrain("test2.da"));
    if (rv) {
        cerr << "error=" << static_cast<int>(rv) << endl;
        return 1;
    }
    cout << "restore.\n" << endl;
    rv = da.build(FileSource("test2.da"));
    if (rv) {
        cerr << "error=" << static_cast<int>(rv) << endl;
        return 1;
    }

    // print array contents.
    const ArrayBody &ab = da.array_body();
    cout << "[0] base=" << ab.base(0, 0) << "(node)" << endl;
    for (size_t i = 1; i < ab.num_entries(); i++) {
        if (ab.is_inuse(i, 0)) {
            cout << "[" << i << "] check=" << ab.check(i, 0)
                 << ", base=" << ab.base(i, 0) << "("
                 << (ab.check(i, 0) == i ? "leaf" : "node") << ")" << endl;
        }
    }
    cout << endl;

    // self common prefix search.
    cout << "self common prefix search:" << endl;
    for (int i = 0; i < count; i++) {
        cout << "  " << keys[i] << ":" << endl;
        DA::Walker w(da, keys[i], keylens[i]);
        do {
            if (w.is_leaf()) {
                cout << "    [" << w.get_leaf_id() << "] "
                     << keys[w.get_leaf_id()] << endl;
            }
        } while (!(rv = w([](const DA::Walker &) { return S_OK; })));
        if (rv != S_BREAK) {
            cout << "  not match - strange..." << endl;
        }
    }

    return 0;
}
