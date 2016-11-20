//
// Copyright (c)2008 Takuya SHIOZAKI,
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

#if !defined(TEST_SEPARATED) && !defined(TEST_STRUCTURED) && \
    !defined(TEST_DELTA_CHECK)
#error TEST_* not defined.
#endif

#if defined(TEST_SEPARATED) || defined(TEST_STRUCTURED)
#ifdef TEST_SEPARATED
#define STORAGE Standard::SeparatedStorage
#else
#define STORAGE Standard::StructuredStorage
#endif
typedef Standard::Traits<char, size_t, unsigned int, STORAGE> TR;
typedef DoubleArray<TR> DA;
typedef AMDA::Standard::SortedKeySource<TR> SortedKeySource;
typedef TR::ArrayBody ArrayBody;
typedef AMDA::Standard::FileDrain<TR> FileDrain;
typedef AMDA::Standard::FileSource<TR> FileSource;
typedef const char *KeyType;
#else // TEST_DELTA_CHECK
typedef DeltaCheck::Traits<size_t, unsigned int> TR;
typedef DoubleArray<TR> DA;
typedef AMDA::DeltaCheck::SortedKeySource<TR> SortedKeySource;
typedef TR::ArrayBody ArrayBody;
typedef AMDA::DeltaCheck::FileDrain<TR> FileDrain;
typedef AMDA::DeltaCheck::FileSource<TR> FileSource;
typedef const AMDA::U8 *KeyType;
#endif


class CPCallback
{
public:
	Status operator () (const DA::Walker &w) const
	{
		printf("    prefix \"%.*s\" found: %u\n",
		       (int)w.depth(), w.key(), w.id());
		return S_OK;
	}
};

void
test_walker(const DA &da, KeyType key, size_t keylen,
	    KeyType subkey =nullptr, size_t subkeylen =0)
{
	DA::Walker w(da, key, keylen);
	Status rv;

	printf("test_walker(%.*s)\n", (int)keylen, key);
retry:
	do {
		printf("  id=%u, depth=%u\n", w.id(), w.depth());
	} while (!(rv = w(CPCallback())));
	if (rv == S_BREAK)
		printf("  found: ");
	else {
		printf("  not found.\n");
		if (rv == S_NO_ENTRY && subkey) {
			printf("  retry by subkey(%.*s)\n",
			       (int)subkeylen, subkey);
			w = DA::Walker(w, subkey, subkeylen);
			subkey = nullptr;
			subkeylen = 0;
			goto retry;
		}
		printf("  error=%d: ", rv);
	}
	printf("id=%u, depth=%u\n", w.id(), w.depth());
}

template <class Policy_>
void
test_common(const char *title, const DA &da, KeyType key, size_t keylen)
{
	printf("%s(%.*s)\n", title, (int)keylen, key);

	DA::Walker w(da, key, keylen);
	Status rv = w.find<Policy_>();

	if (rv == S_NO_ENTRY)
		printf("  not found\n");
	else if (rv)
		printf("  error=%d\n", rv);
	else
		printf("  found: %.*s\n",
		       (int)w.depth(), (const char *)w.key());
}

void test_exact(const DA &da, KeyType key, size_t keylen)
{
	test_common<DA::Walker::ExactPolicy>("test_exact",
					     da, key, keylen);
}

void test_most_common(const DA &da, KeyType key, size_t keylen)
{
	test_common<DA::Walker::MostCommonPolicy>("test_most_common",
						  da, key, keylen);
}

void test_least_common(const DA &da, KeyType key, size_t keylen)
{
	test_common<DA::Walker::LeastCommonPolicy>("test_least_common",
						   da, key, keylen);
}

#define NUM_OF(a)	(sizeof (a) / sizeof (*a))
int
main()
{
	Status rv;
	KeyType keys[] = {
#ifdef TEST_NULL_STRING
		KeyType(""),
#endif
		KeyType("a"),
		KeyType("aa"),
		KeyType("bb"),
		KeyType("bc"),
	};
	size_t keylen[] = {
#ifdef TEST_NULL_STRING
		0,
#endif
		1,
		2,
		2,
		2,
	};
	DA da;

	rv = da.build(SortedKeySource(NUM_OF(keys), keys, keylen));
	if (rv) {
		printf("error=%d\n", (int)rv);
		return 1;
	}

	printf("dump.\n");
	rv = da.dump(FileDrain("test1.da"));
	if (rv) {
		printf("error=%d\n", (int)rv);
		return 1;
	}
	printf("restore.\n");
	rv = da.build(FileSource("test1.da"));
	if (rv) {
		printf("error=%d\n", (int)rv);
		return 1;
	}

	const ArrayBody &ab = da.array_body();
	printf("[0]; base=%u(node)\n", ab.base(0, 0));
	for (size_t i=1; i<ab.num_entries(); i++) {
		if (ab.is_inuse(i, 0)) {
			printf("[%d]; check=%u, base=%u(%s)\n",
			       i, ab.check(i, 0), ab.base(i, 0),
			       ab.check(i, 0)==i?"leaf":"node");
		}
	}

	test_walker(da, KeyType("a"), 1);
	test_walker(da, KeyType("b"), 1);
	test_walker(da, KeyType("aa"), 2);
	test_walker(da, KeyType("ab"), 2, KeyType("a"), 1);
	test_walker(da, KeyType("bb"), 2);
	test_walker(da, KeyType("bc"), 2);
	test_walker(da, KeyType("bcc"), 3, KeyType(""), 0);
	test_walker(da, KeyType("x"), 1);

	test_exact(da, KeyType("a"), 1);
	test_exact(da, KeyType("aa"), 2);
	test_exact(da, KeyType("aaa"), 3);

	test_most_common(da, KeyType("a"), 1);
	test_most_common(da, KeyType("aa"), 2);
	test_most_common(da, KeyType("aaa"), 3);

	test_least_common(da, KeyType("a"), 1);
	test_least_common(da, KeyType("aa"), 2);
	test_least_common(da, KeyType("aaa"), 3);

	return 0;
}
