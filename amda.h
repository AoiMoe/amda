//
// Copyright (c)2017 Takuya SHIOZAKI,
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

//
// static double array.
//

#ifndef AM_DOUBLE_ARRAY_H_
#define AM_DOUBLE_ARRAY_H_

#include <vector>
#include <string>
#include <memory>
#include <algorithm>
#include <type_traits>
#include <utility>
#include <cstdio>
#include <unistd.h>
#ifdef AMDA_DEBUG
#include <cstdlib>
#endif

namespace AMDA {

// ----------------------------------------------------------------------
// fundamental types and utilities
//

using S8 = char;
using U8 = unsigned char;

enum Status {
    S_OK = 0,
    S_BREAK,
    S_NO_ENTRY,
    S_INVALID_KEY,
    S_INVALID_DATA,
    S_IO_ERROR,
    S_NONE,
};

struct Unlinker {
    void operator()(const std::string *file) const {
        if (file)
            unlink(file->c_str());
    }
};

#ifdef AMDA_DEBUG
#define AMDA_ASSERT(cond)                                                                                              \
    do {                                                                                                               \
        if (/*CONSTCOND*/ (cond))                                                                                      \
            ;                                                                                                          \
        else {                                                                                                         \
            std::fprintf(stderr, "assertion failed at %s:%d : \n\t%s\n", __FILE__, __LINE__, #cond);                   \
            std::abort();                                                                                              \
        }                                                                                                              \
    } while (/*CONSTCOND*/ 0)
#else
#define AMDA_ASSERT(cond)
#endif

class NonCopyable {
    NonCopyable(const NonCopyable &) = delete;
    NonCopyable &operator=(const NonCopyable &) = delete;

protected:
    NonCopyable(NonCopyable &&) = default;
    NonCopyable &operator=(NonCopyable &&) = default;
    NonCopyable() = default;
    ~NonCopyable() = default;
};

class NonMovable {
    NonMovable(NonMovable &&) = delete;
    NonMovable &operator=(NonMovable &&) = delete;

protected:
    NonMovable(const NonMovable &) = default;
    NonMovable &operator=(const NonMovable &) = default;
    NonMovable() = default;
    ~NonMovable() = default;
};

template <class Iter_> class IteratorRange {
    Iter_ m_begin, m_end;

public:
    IteratorRange() = delete;
    IteratorRange(Iter_ b, Iter_ e) : m_begin{b}, m_end{e} {}
    Iter_ begin() const { return m_begin; }
    Iter_ end() const { return m_end; }
};

template <class Iter_> IteratorRange<Iter_> make_iter_range(Iter_ b, Iter_ e) { return {b, e}; }

//
// simple Failable type.
//
// see amda_test_failable.cpp to refer usage.
//

template <class> struct Failable;

template <class> struct IsFailable_ { static constexpr bool value = false; };

template <class V_> struct IsFailable_<Failable<V_>> { static constexpr bool value = true; };

class Failure {
public:
    Failure(Status s) : m_status(s) { AMDA_ASSERT(s != S_OK); }
    Status unwrap_failure() const { return m_status; }

private:
    Status m_status;
};

struct Success {};

template <class V_> class Failable : NonCopyable {
    template <class S_> friend class Failable;
    using ValueStorage_ = std::aligned_storage_t<sizeof(V_), alignof(V_)>;

public:
    void reset() {
        if (m_status == S_OK)
            stor_()->~V_();
        m_status = S_NONE;
    }
    ~Failable() { reset(); }
    Failable() {}
    Failable(const V_ &v) = delete;
    Failable &operator=(const V_ &v) = delete;
    Failable(V_ &&v) {
        new (stor_()) V_(std::move(v));
        m_status = S_OK;
    }
    Failable(Failure f) : m_status(f.unwrap_failure()) {}
    template <typename... Args_> Failable(Args_ &&... args) {
        new (stor_()) V_(std::forward<Args_>(args)...);
        m_status = S_OK;
    }
    Failable(Failable &&f) {
        auto s = f.m_status;
        if (f) {
            new (stor_()) V_(std::move(*f.stor_()));
            f.stor_()->~V_();
            f.m_status = S_NONE;
        }
        m_status = s;
    }
    template <typename S_> Failable(Failable<S_> &&f) {
        auto s = f.m_status;
        if (f) {
            new (stor_()) V_(std::move(*f.stor_()));
            f.stor_()->~V_();
            f.m_status = S_NONE;
        }
        m_status = s;
    }
    Failable &operator=(V_ &&v) {
        reset();

        new (stor_()) V_(std::move(v));
        m_status = S_OK;

        return *this;
    }
    Failable &operator=(Failable &&f) {
        reset();

        auto s = f.m_status;
        if (f) {
            new (stor_()) V_(std::move(*f.stor_()));
            f.stor_()->~V_();
            f.m_status = S_NONE;
        }
        m_status = s;

        return *this;
    }
    template <typename S_> Failable &operator=(Failable<S_> &&f) {
        reset();

        auto s = f.m_status;
        if (f) {
            new (stor_()) V_(std::move(*f.stor_()));
            f.stor_()->~V_();
            f.m_status = S_NONE;
        }
        m_status = s;

        return *this;
    }
    Failable &operator=(Status s) {
        AMDA_ASSERT(s != S_OK);
        reset();
        m_status = s;
        return *this;
    }
    //
    // map: it applys f() if S_OK state.
    //
    // f() -> U : Failable<T> -> Failable<U>
    template <class F_>
    auto map(F_ f) -> std::enable_if_t<(!std::is_void<std::result_of_t<F_(V_)>>::value &&
                                        !IsFailable_<std::result_of_t<F_(V_)>>::value &&
                                        !std::is_same<std::result_of_t<F_(V_)>, Failure>::value &&
                                        !std::is_same<std::result_of_t<F_(V_)>, Status>::value),
                                       Failable<std::result_of_t<F_(V_)>>> {
        if (m_status == S_OK) {
            m_status = S_NONE;
            auto ret = f(std::move(*stor_()));
            stor_()->~V_();
            return std::move(ret);
        }
        return Failure{unwrap_failure()};
    }
    // f() -> void : Failable<T> -> Failable<void>
    template <class F_>
    auto map(F_ f) -> std::enable_if_t<std::is_void<std::result_of_t<F_(V_)>>::value, Failable<void>> {
        if (m_status == S_OK) {
            m_status = S_NONE;
            f(std::move(*stor_()));
            stor_()->~V_();
            return Success{};
        }
        return Failure{unwrap_failure()};
    }
    //
    // and_then: it applys f() if S_OK state.
    //
    // f() -> Failable<U> : Failable<T> -> Failable<U>
    template <class F_>
    auto and_then(F_ f) -> std::enable_if_t<IsFailable_<std::result_of_t<F_(V_)>>::value, std::result_of_t<F_(V_)>> {
        if (m_status == S_OK) {
            m_status = S_NONE;
            auto ret = f(std::move(*stor_()));
            stor_()->~V_();
            return std::move(ret);
        }
        return Failure{unwrap_failure()};
    }
    // f() -> Success/Failure : Failable<T> -> Failable<void>
    template <class F_>
    auto and_then(F_ f) -> std::enable_if_t<(std::is_same<std::result_of_t<F_(V_)>, Success>::value ||
                                             std::is_same<std::result_of_t<F_(V_)>, Failure>::value),
                                            Failable<void>> {
        if (m_status == S_OK) {
            m_status = S_NONE;
            auto ret = f(std::move(*stor_()));
            stor_()->~V_();
            return ret;
        }
        return Failure{unwrap_failure()};
    }
    //
    // failure : it applys f() if error state.
    //
    // note : S_NONE is not failure.
    //
    template <class F_> Failable failure(F_ f) {
        if (m_status != S_OK && m_status != S_NONE)
            f(m_status);
        return std::move(*this);
    }
    V_ unwrap() {
        AMDA_ASSERT(m_status == S_OK);
        m_status = S_NONE;
        V_ ret = std::move(*stor_());
        stor_()->~V_();
        return ret;
    }
    Status unwrap_failure() {
        AMDA_ASSERT(m_status != S_OK && m_status != S_NONE);
        return m_status;
    }
    explicit operator bool() const { return m_status == S_OK; }
    bool is_none() const { return m_status == S_NONE; }

private:
    V_ *stor_() { return reinterpret_cast<V_ *>(&m_stor); }
    const V_ *stor_() const { return reinterpret_cast<const V_ *>(&m_stor); }
    Status m_status = S_NONE;
    ValueStorage_ m_stor;
};

template <> class Failable<void> : NonCopyable {
    template <class S_> friend class Failable;

public:
    Failable() = default;
    Failable(Failable &&f) : m_status(f.m_status) {}
    Failable(Success) : m_status(S_OK) {}
    Failable(Failure f) : m_status(f.unwrap_failure()) {}
    explicit Failable(Status s) : m_status(s) {}
    template <class S_> Failable(Failable<S_> s) : m_status(static_cast<Status>(s)) { AMDA_ASSERT(m_status != S_OK); }
    Failable &operator=(Failable &&f) {
        m_status = f.m_status;
        return *this;
    }
    Failable &operator=(Status s) {
        AMDA_ASSERT(s != S_OK);
        m_status = s;
        return *this;
    }
    template <class S_> Failable &operator=(Failable<S_> s) {
        AMDA_ASSERT(static_cast<Status>(s) != S_OK);
        m_status = static_cast<Status>(s);
        return std::move(*this);
    }
    // f() -> U : Failable<T> -> Failable<U>
    template <class F_>
    auto map(F_ f) -> std::enable_if_t<
        (!std::is_void<std::result_of_t<F_()>>::value && !IsFailable_<std::result_of_t<F_()>>::value &&
         !std::is_same<std::result_of_t<F_()>, Failure>::value && !std::is_same<std::result_of_t<F_()>, Status>::value),
        Failable<std::result_of_t<F_()>>> {
        if (m_status == S_OK) {
            m_status = S_NONE;
            return f();
        }
        return Failure{unwrap_failure()};
    }
    // f() -> void : Failable<T> -> Failable<void>
    template <class F_>
    auto map(F_ f) -> std::enable_if_t<std::is_void<std::result_of_t<F_()>>::value, Failable<void>> {
        if (m_status == S_OK) {
            m_status = S_NONE;
            f();
            return S_OK;
        }
        return Failure{unwrap_failure()};
    }
    // f() -> Failable<U> : Failable<T> -> Failable<U>
    template <class F_>
    auto and_then(F_ f) -> std::enable_if_t<IsFailable_<std::result_of_t<F_()>>::value, std::result_of_t<F_()>> {
        if (m_status == S_OK) {
            m_status = S_NONE;
            return f();
        }
        return Failure{unwrap_failure()};
    }
    // f() -> Success/Failure : Failable<T> -> Failable<void>
    template <class F_>
    auto and_then(F_ f) -> std::enable_if_t<(std::is_same<std::result_of_t<F_()>, Success>::value ||
                                             std::is_same<std::result_of_t<F_()>, Failure>::value),
                                            Failable<void>> {
        if (m_status == S_OK) {
            m_status = S_NONE;
            return f();
        }
        return Failure{unwrap_failure()};
    }
    template <class F_> Failable failure(F_ f) {
        if (m_status != S_OK && m_status != S_NONE)
            f(m_status);
        return std::move(*this);
    }
    Status unwrap_failure() {
        AMDA_ASSERT(m_status != S_OK && m_status != S_NONE);
        return m_status;
    }
    explicit operator bool() const { return m_status == S_OK; }
    bool is_none() const { return m_status == S_NONE; }

private:
    Status m_status = S_NONE;
};

template <typename T_> Failable<T_> make_failable(T_ &&arg) { return Failable<T_>{std::move(arg)}; }
template <typename T_> Failable<T_> make_failable(const T_ &arg) { return Failable<T_>{arg}; }
template <typename T_, typename... Args_> Failable<T_> make_failable(Args_ &&... args) {
    return Failable<T_>{std::forward<Args_>(args)...};
}
template <typename T_> Failable<T_> make_failable() { return Failable<T_>{T_{}}; }
template <typename T_> Failable<T_> make_failable(Status s) { return Failable<T_>{s}; }
inline Failable<void> make_failable(Status s) { return Failable<void>{s}; }

// ----------------------------------------------------------------------
// CheckedTraits_
//

template <class Traits_> struct CheckedTraits_ {
    using CharType = typename Traits_::CharType;
    using SizeType = typename Traits_::SizeType;
    using NodeIDType = typename Traits_::NodeIDType;
    using BaseType = typename Traits_::BaseType;
    using CheckType = typename Traits_::CheckType;
    using Storage = typename Traits_::Storage;
    using StorageScratchFactory = typename Storage::ScratchFactory;
    static_assert(std::is_integral<CharType>::value, "");
    static_assert(std::is_integral<SizeType>::value, "");
    static_assert(std::is_integral<NodeIDType>::value, "");
    static_assert(std::is_integral<BaseType>::value, "");
    static_assert(std::is_integral<CheckType>::value, "");
    static constexpr SizeType TERMINATOR = Traits_::TERMINATOR;
    static auto char_to_node_offset(CharType ch) -> SizeType { return Traits_::char_to_node_offset(ch); }
    static auto is_too_dense(SizeType num_filled, SizeType extent) -> bool {
        return Traits_::is_too_dense(num_filled, extent);
    }
    static auto base_to_nid(SizeType idx, BaseType base) -> NodeIDType { return Traits_::base_to_nid(idx, base); }
    static auto nid_to_base(SizeType idx, NodeIDType nid) -> BaseType { return Traits_::nid_to_base(idx, nid); }
    static auto check_to_nid(SizeType idx, CheckType check) -> NodeIDType { return Traits_::check_to_nid(idx, check); }
    static auto nid_to_check(SizeType idx, NodeIDType nid) -> CheckType { return Traits_::nid_to_check(idx, nid); }
    static auto set_inuse(SizeType idx, BaseType *pbase, CheckType *pnid) -> void {
        Traits_::set_inuse(idx, pbase, pnid);
    }
    static auto is_inuse(BaseType base, CheckType check) -> bool { return Traits_::is_inuse(base, check); }
};

// ----------------------------------------------------------------------
// DoubleArray : consumer interface.
//
template <class Traits_> class DoubleArray : NonCopyable {
public:
    class ArrayBody;
    using CheckedTraits = CheckedTraits_<Traits_>;
    using SizeType = typename CheckedTraits::SizeType;
    using CharType = typename CheckedTraits::CharType;
    using NodeIDType = typename CheckedTraits::NodeIDType;
    //
    class Walker;
    //
    template <class Source_> static Failable<DoubleArray> create(const Source_ &src) {
        return Source_::Builder::create(src).map([](auto &&ab) { return DoubleArray{std::move(ab)}; });
    }
    template <class Drain_> Failable<void> dump(Drain_ &drn) const { return drn.dump(m_array_body); }
    template <class Drain_> Failable<void> dump(const Drain_ &drn) const { return drn.dump(m_array_body); }
    const ArrayBody &array_body() const { return m_array_body; }
    void clear() { m_array_body.clear(); }
    template <class Policy_> Failable<Walker> find(const CharType *k, SizeType kl) const {
        Walker w(*this, k, kl);
        if (Status rv = w.template find<Policy_>())
            return Failure{rv};
        return w;
    }
    //
    ~DoubleArray() = default;
    DoubleArray() = default;
    DoubleArray(DoubleArray &&) = default;
    DoubleArray &operator=(DoubleArray &&) = default;

private:
    DoubleArray(ArrayBody &&ab) : m_array_body(std::move(ab)) {}
    ArrayBody m_array_body{};
};

template <class Traits_> class DoubleArray<Traits_>::Walker {
public:
    using CheckedTraits = CheckedTraits_<Traits_>;
    class ExactPolicy;
    class MostCommonPolicy;
    class LeastCommonPolicy;
    ~Walker() = default;
    Walker() = default;
    Walker(const DoubleArray &da, const CharType *k, SizeType kl)
        : m_array_body{&da.m_array_body}, m_key{k}, m_key_length{kl}, m_id{m_array_body->base(0, 0)} {}
    Walker(const Walker &w, const CharType *subkey, SizeType skl)
        : m_array_body{w.m_array_body}, m_key{subkey}, m_key_length{skl}, m_id{w.m_id}, m_depth{0} {}
    template <class CommonPrefixCallback_> Status operator()(CommonPrefixCallback_ cb) {
        if (m_depth > m_key_length)
            return S_BREAK;

        if (auto rv = this->is_leaf() ? cb(*this) : S_OK)
            return rv;

        const auto ofs =
            m_depth == m_key_length ? CheckedTraits::TERMINATOR : CheckedTraits::char_to_node_offset(m_key[m_depth]);

        if (!m_array_body->is_check_ok(m_id, ofs))
            return S_NO_ENTRY;

        m_id = m_array_body->base(m_id, ofs);
        m_depth++;

        return ofs == CheckedTraits::TERMINATOR ? S_BREAK : S_OK;
    }
    template <class Policy_> Status find() { return Policy_::find(*this); }
    Status find_exact() { return this->template find<ExactPolicy>(); }
    Status find_most_common() { return this->template find<MostCommonPolicy>(); }
    Status find_least_common() { return this->template find<LeastCommonPolicy>(); }
    NodeIDType id() const { return m_id; }
    const CharType *key() const { return m_key; }
    SizeType key_length() const { return m_key_length; }
    SizeType depth() const { return std::min(m_depth, m_key_length); }
    bool is_valid() const { return m_array_body != nullptr; }
    bool is_done() const { return m_depth > m_key_length; }
    bool is_leaf() const { return m_array_body->is_check_ok(m_id, CheckedTraits::TERMINATOR); }
    NodeIDType get_leaf_id() const {
        AMDA_ASSERT(this->is_leaf());
        return m_array_body->base(m_id, CheckedTraits::TERMINATOR);
    }

private:
    const ArrayBody *m_array_body = nullptr;
    const CharType *m_key = nullptr;
    SizeType m_key_length = 0;
    NodeIDType m_id = 0;
    SizeType m_depth = 0;
};

template <class Traits_> class DoubleArray<Traits_>::Walker::ExactPolicy {
public:
    static Status find(Walker &w) {
        Status rv;

        while (!(rv = w([](auto) { return S_OK; })))
            ;

        if (rv == S_BREAK)
            rv = S_OK;

        return rv;
    }
};

template <class Traits_> class DoubleArray<Traits_>::Walker::MostCommonPolicy {
public:
    static Status find(Walker &w) {
        Status rv;
        Walker saved;

        while (!(rv = w([&saved](const auto &w) {
                     saved = w;
                     return S_OK;
                 })))
            ;

        switch (rv) {
        case S_BREAK:
            return S_OK;
        case S_NO_ENTRY:
            if (!saved.is_valid())
                return S_NO_ENTRY;
            w = saved;
            return S_OK;
        default:;
        }
        return rv;
    }
};

template <class Traits_> class DoubleArray<Traits_>::Walker::LeastCommonPolicy {
public:
    static Status find(Walker &w) {
        Status rv;

        while (!(rv = w([](const auto &) { return S_BREAK; })))
            ;

        if (rv == S_BREAK)
            rv = S_OK;

        return rv;
    }
};

// ----------------------------------------------------------------------
// ArrayBody : thin wrapper surrounding Storage class.
//
// this class provides common implementation for DoubleArray::Walker class
// to read to backend storage in a layout-independent manner.
//
template <class Traits_> class DoubleArray<Traits_>::ArrayBody : NonCopyable {
public:
    using CheckedTraits = CheckedTraits_<Traits_>;
    using SizeType = typename CheckedTraits::SizeType;
    using NodeIDType = typename CheckedTraits::NodeIDType;
    using Storage = typename CheckedTraits::Storage;

public:
    ~ArrayBody() = default;
    // public interface for class DoubleArray::Walker.
    SizeType num_entries() const { return m_storage.num_entries(); }
    bool is_inuse(NodeIDType nid, SizeType ofs) const {
        ofs += nid;
        return m_storage.is_inuse(ofs);
    }
    NodeIDType check(NodeIDType nid, SizeType ofs) const {
        AMDA_ASSERT(static_cast<SizeType>(nid) + ofs < m_storage.num_entries());
        return m_storage.check(static_cast<SizeType>(nid) + ofs);
    }
    bool is_check_ok(NodeIDType nid, SizeType ofs) const {
        ofs += nid;
        return m_storage.is_inuse(ofs) && m_storage.check(ofs) == nid;
    }
    NodeIDType base(NodeIDType nid, SizeType ofs) const {
        AMDA_ASSERT(static_cast<SizeType>(nid) + ofs < m_storage.num_entries());
        return m_storage.base(static_cast<SizeType>(nid) + ofs);
    }
    // public interface for class DoubleArray.
    void clear() { m_storage.reset(nullptr); }
    ArrayBody() = default;
    ArrayBody(Storage &&s) : m_storage(std::move(s)) {}
    ArrayBody(ArrayBody &&) = default;
    ArrayBody &operator=(ArrayBody &&) = default;
    ArrayBody &operator=(Storage &&s) {
        m_storage = std::move(s);
        return *this;
    }
    const Storage &storage() const { return m_storage; }

private:
    Storage m_storage;
};

// ----------------------------------------------------------------------
// ScratchBuilder : build ArrayBody from scratch.
//
// This class builds an ArrayBody from a "Source_" which is a kind of
// random accessible container containing sorted keys with its leaf identifier.
//
// premise:
//   SizeType Source_::size() const
//     size of the container.
//
//   const __SourceNodeType__ &Source_::operator [] (SizeType index) const
//     array read accessor for the container.
//     index is between 0 and size()-1.
//     __SourceNodeType__ is an arbitrary type relying on the premises
//     described below.
//
//   const SizeType __SourceNodeType__::key_length
//   const CharType __SourceNodeType__::key[]
//     key on the node.
//
//   const NodeIDType __SourceNodeType__::leaf_id
//     id on the node.
//
template <class Traits_, class Source_> class ScratchBuilder : NonCopyable, NonMovable {
public:
    using CheckedTraits = CheckedTraits_<Traits_>;
    using ArrayBody = typename DoubleArray<Traits_>::ArrayBody;
    using SizeType = typename CheckedTraits::SizeType;
    using CharType = typename CheckedTraits::CharType;
    using NodeIDType = typename CheckedTraits::NodeIDType;
    using Storage = typename CheckedTraits::Storage;
    static Failable<ArrayBody> create(const Source_ &src) { return ScratchBuilder{src}.create_(); }

private:
    // thin wrapper surrounding Storage::ScratchFactory class.
    class StorageFactoryWrapper_ : NonCopyable, NonMovable {
    private:
        using StorageFactory_ = typename Storage::ScratchFactory;

    public:
        ~StorageFactoryWrapper_() = default;
        StorageFactoryWrapper_() = default;
        void expand(SizeType sz) { m_base.expand(sz); }
        bool is_inuse(SizeType idx) const { return idx < m_base.num_entries() && m_base.is_inuse(idx); }
        void set_inuse(NodeIDType nid, SizeType ofs) {
            ofs += static_cast<SizeType>(nid);
            AMDA_ASSERT(!m_base.is_inuse(ofs));
            m_base.expand(ofs + 1);
            m_base.set_inuse(ofs);
            m_base.set_check(ofs, nid);
        }
        void set_base(NodeIDType base, SizeType ofs, NodeIDType nid) {
            ofs += static_cast<SizeType>(base);
            this->expand(ofs + 1);
            m_base.set_base(ofs, nid);
        }
        void start() { m_base.start(); }
        ArrayBody done() { return m_base.done(); }

    private:
        StorageFactory_ m_base;
    };

    // Trie:
    //   - a kind of state machine.
    //   - a node indicates a state.
    //     the root is the initial state.
    //   - a edge is corresponding to each char of key.
    // Note: To create a double array, only the subtree of the trie
    //       is necessary because of the depth-first insertion.

    // node.
    class Node_ {
    public:
        ~Node_() = default;
        // uninitialized node.
        Node_() = default;
        // node having edges between [l, r).
        Node_(SizeType l, SizeType r) : m_left{l}, m_right{r} {}
        // left: the first edge, in the array index of
        //       'keys' array.  the actual character code of
        //       the edge is keys[left][this_node_depth].
        const SizeType &left() const { return m_left; }
        SizeType &left() { return m_left; }
        // right: the last edge, similarly to left.
        //        but, this index value points the next key.
        //        thus, this node keeps [left, right) as its own
        //        edges.
        const SizeType &right() const { return m_right; }
        SizeType &right() { return m_right; }
        //
        void set(SizeType l, SizeType r) {
            m_left = l;
            m_right = r;
        }
        //
        SizeType norm() const { return m_right - m_left; }

    private:
        SizeType m_left;
        SizeType m_right;
    };
    // edge.
    class Edge_ {
    public:
        ~Edge_() = default;
        Edge_() = default;
        Edge_(SizeType l, SizeType r, SizeType c) : m_node{l, r}, m_code{c} {}
        // node: child node of the edge.
        Node_ &node() { return m_node; }
        const Node_ &node() const { return m_node; }
        // code: the character code of the edge between this
        //       node and the parent node.
        //       note: this value is converted to the offset form.
        SizeType &code() { return m_code; }
        const SizeType &code() const { return m_code; }

    private:
        Node_ m_node;
        SizeType m_code;
    };
    using EdgeQueue_ = std::vector<Edge_>;
    using UsedNodeIdMask_ = std::vector<bool>;
    //
    Failable<EdgeQueue_> fetch_edges_(const Node_ &parent, SizeType parent_depth) const {
        EdgeQueue_ q;
        AMDA_ASSERT(parent.norm() > 0);

        Edge_ *last_edge = nullptr;
        auto char_of_edge = CheckedTraits::TERMINATOR;
        auto prev_char = CheckedTraits::TERMINATOR;

        for (SizeType i = parent.left(); i < parent.right(); i++) {
            const auto keylen = m_source[i].key_length;
            if (last_edge == nullptr && keylen == parent_depth) {
                AMDA_ASSERT(char_of_edge == CheckedTraits::TERMINATOR);
                AMDA_ASSERT(prev_char == CheckedTraits::TERMINATOR);
                AMDA_ASSERT(i == parent.left());
                // leaf.
            } else if (keylen <= parent_depth) {
                // not sorted nor unique.
                return S_INVALID_KEY;
            } else {
                // node.
                char_of_edge = CheckedTraits::char_to_node_offset(m_source[i].key[parent_depth]);
            }
            if (last_edge == nullptr || char_of_edge != prev_char) {
                // insert a new edge to the queue.
                if (last_edge)
                    last_edge->node().right() = i;
                q.emplace_back(i, i, char_of_edge);
                last_edge = &q.back();
                prev_char = char_of_edge;
            }
            AMDA_ASSERT(last_edge != nullptr);
        }
        AMDA_ASSERT(last_edge != nullptr);
        last_edge->node().right() = parent.right();
        return std::move(q);
    }
    //
    SizeType fit_edges_(const EdgeQueue_ &q) {
        AMDA_ASSERT(!q.empty());

        const auto first_code = q.front().code();
        const auto last_code = q.back().code();

        auto pos = m_next_check_pos;
        auto first = true;
        if (pos < first_code) {
            first = false;
            pos = first_code + 1;
        }

        SizeType node;
        SizeType num_filled = 0;

        // find a fitting position which all elements corresponding to
        // the each edges of the node are free.
        for (;;) {
            if (m_storage_factory.is_inuse(pos)) {
                num_filled++;
                goto retry;
            }
            if (first) {
                first = false;
                m_next_check_pos = pos;
            }
            node = pos - first_code;
            if (m_used_node_id_mask.size() <= node)
                m_used_node_id_mask.resize(node + 1);
            if (m_used_node_id_mask[node]) {
                // node ID must be unique.
                // don't confuse in-used element of the array.
                goto retry;
            }
            m_storage_factory.expand(node + last_code + 1);
            // determine whether the all chars fit to the holes.
            for (const auto e : make_iter_range(q.begin() + 1, q.end())) {
                if (m_storage_factory.is_inuse(node + e.code()))
                    goto retry;
            }
            // now, node ID is determined.
            break;
        retry:
            pos++;
        }
        if (CheckedTraits::is_too_dense(num_filled, pos - m_next_check_pos + 1)) {
            // there are few spaces.  skip the block.
            m_next_check_pos = pos;
        }

        return node;
    }
    //
    Failable<SizeType> insert_edges_(SizeType node_id, const EdgeQueue_ &q, SizeType parent_depth) {

        AMDA_ASSERT(q.size() > 0);
        AMDA_ASSERT(m_used_node_id_mask.size() > node_id);
        AMDA_ASSERT(m_used_node_id_mask[node_id] == false);

        m_used_node_id_mask[node_id] = true;

        // ensure to reserve the elements before recursive call.
        for (const auto &e : q)
            m_storage_factory.set_inuse(node_id, e.code());

        // insert descendants.
        for (const auto &e : q) {
            if (e.code() == CheckedTraits::TERMINATOR) {
                // base[id + ch] expresses the edge to
                // the leaf node.
                AMDA_ASSERT(e.node().norm() == 1);
                AMDA_ASSERT(&e == &*q.begin());
                m_storage_factory.set_base(node_id, CheckedTraits::TERMINATOR, m_source[e.node().left()].leaf_id);
            } else {
                // base[id + ch]  expresses the edge to
                // the other node.
                auto rv = insert_children_(e.node(), parent_depth + 1).map([&](auto cid) {
                    m_storage_factory.set_base(node_id, e.code(), cid);
                });
                if (!rv)
                    return rv.unwrap_failure();
            }
        }
        return node_id;
    }
    Failable<SizeType> insert_children_(const Node_ &parent, SizeType parent_depth) {
        return fetch_edges_(parent, parent_depth).and_then([&](auto q) {
            return this->insert_edges_(this->fit_edges_(q), q, parent_depth);
        });
    }
    Failable<ArrayBody> create_() {
        if (m_source.size() == 0)
            return Failure{S_NO_ENTRY};

        m_next_check_pos = 1;
        m_used_node_id_mask.resize(1);
        m_used_node_id_mask[0] = true;

        m_storage_factory.start();

        return insert_children_(Node_{0, m_source.size()}, 0).map([&](auto node_id) {
            // set base[0] to the root node ID, which should be 1.
            // note: node #0 is not a valid node.
            AMDA_ASSERT(node_id != 0);
            m_storage_factory.set_base(0, CheckedTraits::TERMINATOR, node_id);
            return m_storage_factory.done();
        });
    }
    ScratchBuilder(const Source_ &src) : m_source{src} {}
    ~ScratchBuilder() = default;
    //
    const Source_ &m_source;
    StorageFactoryWrapper_ m_storage_factory;
    SizeType m_next_check_pos = 0;
    // already used node IDs.
    UsedNodeIdMask_ m_used_node_id_mask;
};

// ----------------------------------------------------------------------
// PersistBuilder : build ArrayBody from secondary storage.
//
template <class Traits_> class PersistBuilder {
public:
    using ArrayBody = typename DoubleArray<Traits_>::ArrayBody;

public:
    template <class Source_> static Failable<ArrayBody> create(const Source_ &src) { return src.load(); }
};

// ----------------------------------------------------------------------
// Standard implementation for normal usage.
//

namespace Standard {

// ----------------------------------------------------------------------
// Source for ScratchBuilder.
//
// prerequisite: key is sorted by dictionary order.
//

//
// Separated source.
//
template <class Traits_> class SeparatedScratchSource {
public:
    using CheckedTraits = CheckedTraits_<Traits_>;
    using CharType = typename CheckedTraits::CharType;
    using SizeType = typename CheckedTraits::SizeType;
    using NodeIDType = typename CheckedTraits::NodeIDType;
    using Builder = ScratchBuilder<Traits_, SeparatedScratchSource>;

private:
    using KeyType_ = const CharType *;
    struct Wrapper_ {
        NodeIDType leaf_id;
        KeyType_ key;
        SizeType key_length;
    };
    NodeIDType leaf_id_(SizeType idx) const { return m_leaf_ids ? m_leaf_ids[idx] : static_cast<NodeIDType>(idx); }

public:
    ~SeparatedScratchSource() = default;
    SeparatedScratchSource(SizeType ne, const KeyType_ *k, const SizeType *kl, const NodeIDType *lid = nullptr)
        : m_size{ne}, m_keys{k}, m_key_lengths{kl}, m_leaf_ids{lid} {}
    template <SizeType ne>
    SeparatedScratchSource(const CharType (&k)[ne], const SizeType (&kl)[ne])
        : m_size{ne}, m_keys{k}, m_key_lengths{kl} {}
    template <SizeType ne>
    SeparatedScratchSource(const CharType (&k)[ne], const SizeType (&kl)[ne], const NodeIDType (&lid)[ne])
        : m_size{ne}, m_keys{k}, m_key_lengths{kl}, m_leaf_ids{lid} {}
    SizeType size() const { return m_size; }
    const Wrapper_ operator[](SizeType idx) const {
        AMDA_ASSERT(idx >= 0 && idx < m_size);
        return {.leaf_id = leaf_id_(idx), .key = m_keys[idx], .key_length = m_key_lengths[idx]};
    }

private:
    SizeType m_size;
    const KeyType_ *m_keys;
    const SizeType *m_key_lengths;
    const NodeIDType *m_leaf_ids = nullptr;
};

//
// Structured source.
//
template <class Traits_, class Element_> class StructuredScratchSource {
public:
    using CheckedTraits = CheckedTraits_<Traits_>;
    using CharType = typename CheckedTraits::CharType;
    using SizeType = typename CheckedTraits::SizeType;
    using NodeIDType = typename CheckedTraits::NodeIDType;
    using Builder = ScratchBuilder<Traits_, StructuredScratchSource>;

private:
    using KeyType_ = const CharType *;

public:
    ~StructuredScratchSource() = default;
    StructuredScratchSource(SizeType ne, const Element_ *e) : m_size{ne}, m_elements{e} {}
    template <class Container_> StructuredScratchSource(const Container_ &e) : m_size{e.size()}, m_elements{&e[0]} {}
    template <SizeType ne> StructuredScratchSource(const Element_ (&e)[ne]) : m_size{ne}, m_elements{e} {}
    SizeType size() const { return m_size; }
    const Element_ &operator[](SizeType idx) const { return m_elements[idx]; }

private:
    SizeType m_size;
    const Element_ *m_elements;
};

// ----------------------------------------------------------------------

template <class, class> class FileAccessorTmpl;

template <class Traits_> class FileSource {
private:
    using ArrayBody_ = typename DoubleArray<Traits_>::ArrayBody;
    using Storage_ = typename Traits_::Storage;
    using Accessor_ = FileAccessorTmpl<Traits_, Storage_>;

public:
    using Builder = PersistBuilder<Traits_>;
    //
    ~FileSource() = default;
    FileSource(const std::string &fn) : m_filename{fn} {}
    Failable<ArrayBody_> load() const { return Accessor_::load(m_filename); }

private:
    std::string m_filename;
};

template <class Traits_> class FileDrain {
private:
    using ArrayBody_ = typename DoubleArray<Traits_>::ArrayBody;
    using Storage_ = typename Traits_::Storage;
    using Accessor_ = FileAccessorTmpl<Traits_, Storage_>;

public:
    ~FileDrain() = default;
    FileDrain(const std::string &fn) : m_filename{fn} {}
    Failable<void> dump(const ArrayBody_ &body) const { return Accessor_::save(m_filename, body); }

private:
    std::string m_filename;
};

// ----------------------------------------------------------------------
// storage type which separates base and check into individual arrays.
//

template <class Traits_> class SeparatedStorage : NonCopyable {
public:
    using CheckedTraits = CheckedTraits_<Traits_>;
    using SizeType = typename CheckedTraits::SizeType;
    using NodeIDType = typename CheckedTraits::NodeIDType;
    using BaseType = typename CheckedTraits::BaseType;
    using CheckType = typename CheckedTraits::CheckType;
    class ScratchFactory;
    class HouseKeeper {
    public:
        virtual ~HouseKeeper() = 0;
        virtual SizeType num_entries() const = 0;
        virtual const BaseType *bases() const = 0;
        virtual const CheckType *checks() const = 0;
    };
    //
    ~SeparatedStorage() = default;
    SeparatedStorage() = default;
    SeparatedStorage(SeparatedStorage &&) = default;
    SeparatedStorage &operator=(SeparatedStorage &&) = default;
    explicit SeparatedStorage(HouseKeeper *hk) { reset(hk); }
    void reset(HouseKeeper *hk = nullptr) {
        m_house_keeper.reset(hk);
        if (hk) {
            m_num_entries = hk->num_entries();
            m_bases = hk->bases();
            m_checks = hk->checks();
        }
    }
    SizeType num_entries() const { return m_num_entries; }
    NodeIDType base(SizeType idx) const { return CheckedTraits::base_to_nid(idx, m_bases[idx]); }
    NodeIDType check(SizeType idx) const { return CheckedTraits::check_to_nid(idx, m_checks[idx]); }
    bool is_inuse(SizeType idx) const { return CheckedTraits::is_inuse(m_bases[idx], m_checks[idx]); }
    const HouseKeeper &house_keeper() const { return *m_house_keeper; }

private:
    std::unique_ptr<HouseKeeper> m_house_keeper;
    SizeType m_num_entries = 0;
    const BaseType *m_bases = nullptr;
    const CheckType *m_checks = nullptr;
};
template <class Traits_> SeparatedStorage<Traits_>::HouseKeeper::~HouseKeeper() {}

template <class Traits_> class SeparatedStorage<Traits_>::ScratchFactory : NonCopyable, NonMovable {
private:
    using CheckedTraits = CheckedTraits_<Traits_>;
    using BaseArray_ = std::vector<BaseType>;
    using CheckArray_ = std::vector<CheckType>;
    class VariableSizedHouseKeeper_ final : public HouseKeeper {
        friend class ScratchFactory;

    public:
        // HouseKeeper interface
        ~VariableSizedHouseKeeper_() override = default;
        SizeType num_entries() const override { return m_num_entries; }
        const BaseType *bases() const override { return &m_bases[0]; }
        const CheckType *checks() const override { return &m_checks[0]; }
        //
        VariableSizedHouseKeeper_() = default;

    private:
        SizeType m_num_entries = 0;
        BaseArray_ m_bases;
        CheckArray_ m_checks;
    };
    //
    SizeType &num_entries_() { return m_house_keeper->m_num_entries; }
    SizeType num_entries_() const { return m_house_keeper->m_num_entries; }
    BaseArray_ &bases_() { return m_house_keeper->m_bases; }
    const BaseArray_ &bases_() const { return m_house_keeper->m_bases; }
    CheckArray_ &checks_() { return m_house_keeper->m_checks; }
    const CheckArray_ &checks_() const { return m_house_keeper->m_checks; }

public:
    // for ScratchBuilder interface
    SizeType num_entries() const { return this->num_entries_(); }
    void set_base(SizeType idx, NodeIDType nid) { this->bases_()[idx] = CheckedTraits::nid_to_base(idx, nid); }
    NodeIDType base(SizeType idx) const { return CheckedTraits::base_to_nid(idx, this->bases_()[idx]); }
    void set_check(SizeType idx, NodeIDType nid) { this->checks_()[idx] = CheckedTraits::nid_to_check(idx, nid); }
    NodeIDType check(SizeType idx) const { return CheckedTraits::check_to_nid(idx, this->checks_()[idx]); }
    void set_inuse(SizeType idx) { CheckedTraits::set_inuse(idx, &this->bases_()[idx], &this->checks_()[idx]); }
    bool is_inuse(SizeType idx) const { return CheckedTraits::is_inuse(this->bases_()[idx], this->checks_()[idx]); }
    void expand(SizeType s) {
        if (s > this->num_entries_()) {
            this->bases_().resize(s, 0);
            this->checks_().resize(s, 0);
            this->num_entries_() = s;
        }
    }
    void start() { m_house_keeper.reset(new VariableSizedHouseKeeper_); }
    SeparatedStorage done() { return SeparatedStorage{m_house_keeper.release()}; }
    //
    ~ScratchFactory() = default;
    ScratchFactory() = default;

private:
    std::unique_ptr<VariableSizedHouseKeeper_> m_house_keeper;
};

template <class Traits_> class FileAccessorTmpl<Traits_, SeparatedStorage<Traits_>> {
public:
    using CheckedTraits = CheckedTraits_<Traits_>;
    using SizeType = typename CheckedTraits::SizeType;
    using BaseType = typename CheckedTraits::BaseType;
    using CheckType = typename CheckedTraits::CheckType;

private:
    using ArrayBody_ = typename DoubleArray<Traits_>::ArrayBody;
    using Storage_ = SeparatedStorage<Traits_>;
    using HouseKeeper_ = typename Storage_::HouseKeeper;
    class FixedSizedHouseKeeper_ final : public HouseKeeper_ {
    public:
        // for HouseKeeper interface
        ~FixedSizedHouseKeeper_() override = default;
        SizeType num_entries() const override { return m_num_entries; }
        const BaseType *bases() const override { return m_bases.get(); }
        const CheckType *checks() const override { return m_checks.get(); }
        //
        FixedSizedHouseKeeper_(SizeType n) : m_num_entries{n}, m_bases{new BaseType[n]}, m_checks{new CheckType[n]} {}
        BaseType *bases() { return m_bases.get(); }
        CheckType *checks() { return m_checks.get(); }

    private:
        SizeType m_num_entries;
        std::unique_ptr<BaseType[]> m_bases;
        std::unique_ptr<CheckType[]> m_checks;
    };

public:
    // XXX: machine dependent.
    static Failable<void> save(const std::string &fn, const ArrayBody_ &body) {
        std::unique_ptr<std::FILE, decltype(&fclose)> fp(std::fopen(fn.c_str(), "wb"), &std::fclose);

        if (!fp)
            return Failure{S_IO_ERROR};

        std::unique_ptr<const std::string, Unlinker> unlinker(&fn);
        const HouseKeeper_ &hk = body.storage().house_keeper();
        SizeType ne = hk.num_entries();

        if (std::fwrite(&ne, sizeof(SizeType), 1, fp.get()) != 1)
            return Failure{S_IO_ERROR};
        if (std::fwrite(hk.bases(), sizeof(BaseType), ne, fp.get()) != ne)
            return Failure{S_IO_ERROR};
        if (std::fwrite(hk.checks(), sizeof(CheckType), ne, fp.get()) != ne)
            return Failure{S_IO_ERROR};

        unlinker.release();

        return Success{};
    }
    // XXX: machine dependent.
    static Failable<ArrayBody_> load(const std::string &fn) {

        std::unique_ptr<std::FILE, decltype(&std::fclose)> fp(std::fopen(fn.c_str(), "rb"), &std::fclose);

        if (!fp)
            return Failure{S_NO_ENTRY};

        SizeType ne;
        if (std::fread(&ne, sizeof(ne), 1, fp.get()) != 1 || ne == 0)
            return Failure{S_INVALID_DATA};

        std::unique_ptr<FixedSizedHouseKeeper_> hk(new FixedSizedHouseKeeper_(ne));

        if (std::fread(hk->bases(), sizeof(BaseType), ne, fp.get()) != ne)
            return Failure{S_INVALID_DATA};
        if (std::fread(hk->checks(), sizeof(CheckType), ne, fp.get()) != ne)
            return Failure{S_INVALID_DATA};

        return ArrayBody_{Storage_{hk.release()}};
    }
};

// ----------------------------------------------------------------------
// storage type which stores base and check into a structure of
// the array element.
//

template <class Traits_> class StructuredStorage : NonCopyable {
public:
    using CheckedTraits = CheckedTraits_<Traits_>;
    using SizeType = typename CheckedTraits::SizeType;
    using NodeIDType = typename CheckedTraits::NodeIDType;
    using BaseType = typename CheckedTraits::BaseType;
    using CheckType = typename CheckedTraits::CheckType;
    using ElementType = typename Traits_::ElementType;
    class ScratchFactory;
    class HouseKeeper {
    public:
        virtual ~HouseKeeper() = 0;
        virtual SizeType num_entries() const = 0;
        virtual const ElementType *elements() const = 0;
    };
    //
    ~StructuredStorage() = default;
    StructuredStorage() = default;
    StructuredStorage(StructuredStorage &&) = default;
    StructuredStorage &operator=(StructuredStorage &&) = default;
    explicit StructuredStorage(HouseKeeper *hk) { reset(hk); }
    void reset(HouseKeeper *hk = nullptr) {
        m_house_keeper.reset(hk);
        if (hk) {
            m_num_entries = hk->num_entries();
            m_elements = hk->elements();
        }
    }
    SizeType num_entries() const { return m_num_entries; }
    NodeIDType base(SizeType idx) const { return CheckedTraits::base_to_nid(idx, m_elements[idx].base); }
    NodeIDType check(SizeType idx) const { return CheckedTraits::check_to_nid(idx, m_elements[idx].check); }
    bool is_inuse(SizeType idx) const { return CheckedTraits::is_inuse(m_elements[idx].base, m_elements[idx].check); }
    const HouseKeeper &house_keeper() const { return *m_house_keeper; }

private:
    std::unique_ptr<HouseKeeper> m_house_keeper;
    SizeType m_num_entries = 0;
    const ElementType *m_elements = nullptr;
};
template <class Traits_> StructuredStorage<Traits_>::HouseKeeper::~HouseKeeper() {}

template <class Traits_> class StructuredStorage<Traits_>::ScratchFactory : NonCopyable, NonMovable {
private:
    using CheckedTraits = CheckedTraits_<Traits_>;
    using ElementArray_ = std::vector<ElementType>;
    class VariableSizedHouseKeeper_ final : public HouseKeeper {
        friend class ScratchFactory;

    public:
        // HouseKeeper interface
        ~VariableSizedHouseKeeper_() override = default;
        SizeType num_entries() const override { return m_num_entries; }
        const ElementType *elements() const override { return &m_elements[0]; }
        //
        VariableSizedHouseKeeper_() = default;

    private:
        SizeType m_num_entries = 0;
        ElementArray_ m_elements;
    };
    //
    SizeType &num_entries_() { return m_house_keeper->m_num_entries; }
    SizeType num_entries_() const { return m_house_keeper->m_num_entries; }
    ElementArray_ &elements_() { return m_house_keeper->m_elements; }
    const ElementArray_ &elements_() const { return m_house_keeper->m_elements; }

public:
    // for ScratchBuilder interface
    SizeType num_entries() const { return this->num_entries_(); }
    void set_base(SizeType idx, NodeIDType nid) { this->elements_()[idx].base = CheckedTraits::nid_to_base(idx, nid); }
    NodeIDType base(SizeType idx) const { return CheckedTraits::base_to_nid(idx, this->elements_()[idx].base); }
    void set_check(SizeType idx, NodeIDType nid) {
        this->elements_()[idx].check = CheckedTraits::nid_to_check(idx, nid);
    }
    NodeIDType check(SizeType idx) const { return CheckedTraits::check_to_nid(idx, this->elements_()[idx].check); }
    void set_inuse(SizeType idx) {
        CheckedTraits::set_inuse(idx, &this->elements_()[idx].base, &this->elements_()[idx].check);
    }
    bool is_inuse(SizeType idx) const {
        return CheckedTraits::is_inuse(this->elements_()[idx].base, this->elements_()[idx].check);
    }
    void expand(SizeType s) {
        if (s > this->num_entries_()) {
            static const ElementType z = {0, 0};
            this->elements_().resize(s, z);
            this->num_entries_() = s;
        }
    }
    void start() { m_house_keeper.reset(new VariableSizedHouseKeeper_); }
    StructuredStorage done() { return StructuredStorage{m_house_keeper.release()}; }
    //
    ~ScratchFactory() = default;
    ScratchFactory() = default;

private:
    std::unique_ptr<VariableSizedHouseKeeper_> m_house_keeper;
};

template <class Traits_> class FileAccessorTmpl<Traits_, StructuredStorage<Traits_>> {
public:
    using CheckedTraits = CheckedTraits_<Traits_>;
    using SizeType = typename CheckedTraits::SizeType;
    using ElementType = typename Traits_::ElementType;

private:
    using ArrayBody_ = typename DoubleArray<Traits_>::ArrayBody;
    using Storage_ = StructuredStorage<Traits_>;
    using HouseKeeper_ = typename Storage_::HouseKeeper;
    class FixedSizedHouseKeeper_ final : public HouseKeeper_ {
    public:
        // for HouseKeeper interface
        ~FixedSizedHouseKeeper_() override = default;
        SizeType num_entries() const override { return m_num_entries; }
        const ElementType *elements() const override { return m_elements.get(); }
        //
        FixedSizedHouseKeeper_(SizeType n) : m_num_entries{n}, m_elements{new ElementType[n]} {}
        ElementType *elements() { return m_elements.get(); }

    private:
        SizeType m_num_entries;
        std::unique_ptr<ElementType[]> m_elements;
    };

public:
    // XXX: machine dependent.
    static Failable<void> save(const std::string &fn, const ArrayBody_ &body) {
        std::unique_ptr<std::FILE, decltype(&std::fclose)> fp(std::fopen(fn.c_str(), "wb"), &std::fclose);

        if (!fp)
            return Failure{S_IO_ERROR};

        std::unique_ptr<const std::string, Unlinker> unlinker(&fn);

        const HouseKeeper_ &hk = body.storage().house_keeper();
        SizeType ne = hk.num_entries();

        if (std::fwrite(&ne, sizeof(SizeType), 1, fp.get()) != 1)
            return Failure{S_IO_ERROR};
        if (std::fwrite(hk.elements(), sizeof(ElementType), ne, fp.get()) != ne)
            return Failure{S_IO_ERROR};

        unlinker.release();

        return Success{};
    }
    // XXX: machine dependent.
    static Failable<ArrayBody_> load(const std::string &fn) {

        std::unique_ptr<std::FILE, decltype(&std::fclose)> fp(std::fopen(fn.c_str(), "rb"), &std::fclose);

        if (!fp)
            return Failure{S_IO_ERROR};

        SizeType ne;
        if (std::fread(&ne, sizeof(ne), 1, fp.get()) != 1 || ne == 0)
            return Failure{S_IO_ERROR};

        std::unique_ptr<FixedSizedHouseKeeper_> hk(new FixedSizedHouseKeeper_(ne));

        if (std::fread(hk->elements(), sizeof(ElementType), ne, fp.get()) != ne)
            return Failure{S_IO_ERROR};

        return ArrayBody_{Storage_{hk.release()}};
    }
};

// ----------------------------------------------------------------------

template <typename BaseType_, typename CheckType_> struct Element {
    BaseType_ base;
    CheckType_ check;
};

// ----------------------------------------------------------------------

template <typename CharType_, typename SizeType_, typename NodeIDType_, template <class> class StorageType_,
          typename BaseType_ = NodeIDType_, typename CheckType_ = NodeIDType_,
          class ElementType_ = Element<BaseType_, CheckType_>>
struct Traits {
    using CharType = CharType_;
    using SizeType = SizeType_;
    using NodeIDType = NodeIDType_;
    using Storage = StorageType_<Traits>;
    // the node offset for the terminator.
    static constexpr SizeType_ TERMINATOR = 0;
    // convert character code to corresponding node offset.
    static SizeType_ char_to_node_offset(CharType ch) { return static_cast<SizeType_>(ch) + 1; }
    // whether the zone is too dense.
    static bool is_too_dense(SizeType num_filled, SizeType extent) {
        return static_cast<float>(num_filled) / extent >= 0.95;
    }
    // for Storage
    using BaseType = BaseType_;
    using CheckType = CheckType_;
    using ElementType = ElementType_; // only for StructuredStorage
    static NodeIDType_ base_to_nid(SizeType_, BaseType_ v) { return NodeIDType(v); }
    static BaseType_ nid_to_base(SizeType_, NodeIDType_ v) { return BaseType(v); }
    static NodeIDType_ check_to_nid(SizeType_, CheckType_ v) { return NodeIDType(v); }
    static CheckType_ nid_to_check(SizeType_, NodeIDType_ v) { return CheckType(v); }
    static void set_inuse(SizeType_, BaseType_ *, CheckType_ *) {
        // it is done by setting to check array.
    }
    static bool is_inuse(BaseType_, CheckType_ chk) { return chk != 0; }
};

} // namespace Standard

// ----------------------------------------------------------------------
// Delta Check implementation.
//
// Assumptions:
//   - CharType is 8bit.
//   - \0 is a terminator and never included in valid strings.
//
// check[id+ch] == id if node `id' has a edge corresponding to `ch',
// and 0x01 <= ch <= 0xFF here.
// Thus, check[index] can takes from index-0xFF to index-0x01.
// Additionary, check[index] may take 0 if ch == 0 means the terminator.
//
// The check array can be expressed as an array of 8bit values:
//   deltaCheck[index] = index - check[index]
//   (deltaCheck[id+ch] = id+ch - id = ch)
//
// To restore check[index] from deltaCheck[index]:
//   check[index] = index - deltaCheck[index]
//
// base[] == 0 can be used to indicate the unexistence of an edge instead
// of check[] == 0.
//
// structured storage seems not fit because of alignment.
//

namespace DeltaCheck {

using Standard::FileDrain;
using Standard::FileSource;
using Standard::SeparatedScratchSource;
using Standard::SeparatedStorage;
using Standard::StructuredScratchSource;

template <typename SizeType_, typename NodeIDType_, typename BaseType_ = NodeIDType_> struct Traits {
    using CharType = U8;
    using SizeType = SizeType_;
    using NodeIDType = NodeIDType_;
    using Storage = SeparatedStorage<Traits>;
    static constexpr SizeType_ TERMINATOR = 0;
    static SizeType_ char_to_node_offset(CharType ch) { return static_cast<SizeType_>(ch) + 1; }
    static bool is_too_dense(SizeType num_filled, SizeType extent) {
        return static_cast<float>(num_filled) / extent >= 0.95;
    }
    using BaseType = BaseType_;
    using CheckType = U8;
    static NodeIDType_ base_to_nid(SizeType_, BaseType_ v) {
        AMDA_ASSERT(v != 0);
        return NodeIDType_(v) - 1;
    }
    static BaseType_ nid_to_base(SizeType_, NodeIDType_ v) { return BaseType_(v + 1); }
    static NodeIDType_ check_to_nid(SizeType_ idx, CheckType v) { return NodeIDType_(idx - SizeType_(v)); }
    static CheckType nid_to_check(SizeType_ idx, NodeIDType_ v) { return U8(idx - SizeType_(v)); }
    static void set_inuse(SizeType_, BaseType_ *pb, CheckType *) {
        *pb = 1; // dummy to indicate in-used.
    }
    static bool is_inuse(BaseType_ b, CheckType) { return b != 0; }
};

} // namespace DeltaCheck

} // namespace AMDA

#endif
