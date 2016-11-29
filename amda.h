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

namespace AMDA
{

// ----------------------------------------------------------------------
// utilities
//

enum Status
{
    S_OK = 0,
    S_BREAK,
    S_NO_ENTRY,
    S_INVALID_KEY,
    S_INVALID_DATA,
    S_IO_ERROR,
    S_NONE,
};

struct Unlinker
{
    void operator () (const std::string *file) const
    {
        if (file) unlink(file->c_str());
    }
};

// XXX
using S8 = char;
using U8 = unsigned char;

#ifdef AMDA_DEBUG
#define AMDA_ASSERT(cond)                                               \
    do {                                                                \
        if (/*CONSTCOND*/(cond))                                        \
            ;                                                           \
        else {                                                          \
            std::fprintf(stderr,                                        \
                         "assertion failed at %s:%d : \n\t%s\n",        \
                         __FILE__, __LINE__, #cond);                    \
            std::abort();                                               \
        }                                                               \
    } while (/*CONSTCOND*/0)
#else
#define AMDA_ASSERT(cond)
#endif

class NonCopyable
{
    NonCopyable(const NonCopyable &) = delete;
    NonCopyable &operator = (const NonCopyable &) = delete;
protected:
    NonCopyable(NonCopyable &&) = default;
    NonCopyable &operator = (NonCopyable &&) = default;
    NonCopyable() = default;
    ~NonCopyable() = default;
};

class NonMovable
{
    NonMovable(NonMovable &&) = delete;
    NonMovable &operator = (NonMovable &&) = delete;
protected:
    NonMovable(const NonMovable &) = default;
    NonMovable &operator = (const NonMovable &) = default;
    NonMovable() = default;
    ~NonMovable() = default;
};

template <class Iter_>
class IteratorView
{
    Iter_ m_begin, m_end;
public:
    IteratorView() = delete;
    IteratorView(Iter_ b, Iter_ e) : m_begin{b}, m_end{e} { }
    Iter_ begin() const { return m_begin; }
    Iter_ end() const { return m_end; }
};

template <class Iter_>
IteratorView<Iter_> make_iter_view(Iter_ b, Iter_ e)
{
    return { b, e };
}

//
// simple Failable type.
//
template <class V_>
class Failable : NonCopyable
{
    using ValueStorage_ = std::aligned_storage_t<sizeof (V_), alignof (V_)>;
public:
    void reset()
    {
        if (m_status == S_OK)
            stor_()->~V_();
        m_status = S_NONE;
    }
    ~Failable()
    {
        reset();
    }
    Failable() { }
    Failable(const V_ &v) = delete;
    Failable &operator = (const V_ &v) = delete;
    Failable(V_ &&v)
    {
        new (stor_()) V_(std::move(v));
        m_status = S_OK;
    }
    template <typename ...Args_>
    Failable(Args_ &&...args)
    {
        new (stor_()) V_(std::forward<Args_>(args)...);
        m_status = S_OK;
    }
    Failable(Failable &&f)
    {
        auto s = f.m_status;
        if (f) {
            new (stor_()) V_(std::move(*f.stor_()));
            f.stor_()->~V_();
            f.m_status = S_NONE;
        }
        m_status = s;
    }
    Failable(Status s) : m_status(s) { AMDA_ASSERT(s != S_OK); }
    template <class S_>
    Failable(Failable<S_> s) : m_status(static_cast<Status>(s))
    {
        AMDA_ASSERT(m_status != S_OK);
    }
    Failable &operator = (Failable &&f)
    {
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
    Failable &operator = (V_ &&v)
    {
        reset();

        new (stor_()) V_(std::move(v));
        m_status = S_OK;

        return *this;
    }
    Failable &operator = (Status s)
    {
        AMDA_ASSERT(s != S_OK);
        reset();
        m_status = s;
        return *this;
    }
    template <class S_>
    Failable &operator = (Failable<S_> s)
    {
        AMDA_ASSERT(static_cast<Status>(s) != S_OK);
        reset();
        m_status = static_cast<Status>(s);
        return *this;
    }
    template <class F_>
    auto apply(F_ f) ->
        std::enable_if_t<
        (!std::is_void<decltype (f(std::move(*(V_*)nullptr)))>::value &&
         !std::is_same<decltype (f(std::move(*(V_*)nullptr))), Status>::value),
        Failable<decltype (f(*(V_*)nullptr))>>
    {
        if (m_status == S_OK) {
            m_status = S_NONE;
            auto ret = f(std::move(*stor_()));
            stor_()->~V_();
            return ret;
        }
        return std::move(*this);
    }
    template <class F_>
    auto apply(F_ f) ->
        std::enable_if_t<
        std::is_same<decltype (f(std::move(*(V_*)nullptr))), Status>::value,
        Failable<void>>
    {
        if (m_status == S_OK) {
            m_status = S_NONE;
            auto ret = f(std::move(*stor_()));
            stor_()->~V_();
            return ret;
        }
        return std::move(*this);
    }
    template <class F_>
    auto apply(F_ f) ->
        std::enable_if_t<
        std::is_void<decltype (f(std::move(*(V_*)nullptr)))>::value,
        Failable<decltype (f(std::move(*(V_*)nullptr)))>>
    {
        if (m_status == S_OK) {
            m_status = S_NONE;
            f(std::move(*stor_()));
            stor_()->~V_();
        }
        return std::move(*this);
    }
    template <class F_>
    Failable failure(F_ f)
    {
        if (m_status != S_OK && m_status != S_NONE)
            f(m_status);
        return std::move(*this);
    }
    V_ unwrap()
    {
        AMDA_ASSERT(m_status == S_OK);
        m_status = S_NONE;
        V_ ret = std::move(*stor_());
        stor_()->~V_();
        return ret;
    }
    explicit operator bool () const { return m_status == S_OK; }
    operator Status () const { return m_status; }
    bool is_none() const { return m_status == S_NONE; }
private:
    V_ *stor_() { return (V_ *)(void *)&m_stor; }
    const V_ *stor_() const { return (const V_ *)(const void *)&m_stor; }
    Status m_status = S_NONE;
    ValueStorage_ m_stor;
};

template <>
class Failable<void> : NonCopyable
{
public:
    Failable() = default;
    Failable(Failable &&f) : m_status(f.m_status) { }
    Failable(Status s) : m_status(s) { AMDA_ASSERT(s != S_OK); }
    template <class S_>
    Failable(Failable<S_> s) : m_status(static_cast<Status>(s))
    {
        AMDA_ASSERT(m_status != S_OK);
    }
    Failable &operator = (Failable &&f)
    {
        m_status = f.m_status;
        return *this;
    }
    Failable &operator = (Status s)
    {
        AMDA_ASSERT(s != S_OK);
        m_status = s;
        return *this;
    }
    template <class S_>
    Failable &operator = (Failable<S_> s)
    {
        AMDA_ASSERT(static_cast<Status>(s) != S_OK);
        m_status = static_cast<Status>(s);
        return *this;
    }
    template <class F_>
    Failable failure(F_ f)
    {
        if (m_status != S_OK && m_status != S_NONE)
            f(m_status);
        return std::move(*this);
    }
    // Because Failable<void> is always failed,
    // apply() and operator bool() is deleted.
    template <class F_>
    auto apply(F_ f) -> void = delete;
    explicit operator bool () const = delete;
    operator Status () const { return m_status; }
    bool is_none() const { return m_status == S_NONE; }
private:
    Status m_status = S_NONE;
};

template <typename T_, typename ...Args_>
Failable<T_> make_failable(Args_ && ...args)
{
    return Failable<T_>{std::forward<Args_>(args)...};
}
template <typename T_>
Failable<T_> make_failable()
{
    return Failable<T_>{T_{}};
}


// ----------------------------------------------------------------------

//
// DoubleArray : consumer interface.
//
template <class Traits_>
class DoubleArray : NonCopyable, NonMovable
{
public:
    using ArrayBody = typename Traits_::ArrayBody;
    using SizeType = typename Traits_::SizeType;
    using CharType = typename Traits_::CharType;
    using NodeIDType = typename Traits_::NodeIDType;
    //
    class Walker;
    //
    template <class Source_>
    Status build(const Source_ &src)
    {
        auto fa = Source_::Builder::build(src);

        if (!fa)
            return fa;
        m_array_body = std::move(fa.unwrap());
        return S_OK;
    }
    template <class Drain_>
    Status dump(Drain_ &drn) const
    {
        return drn.dump(m_array_body);
    }
    template <class Drain_>
    Status dump(const Drain_ &drn) const
    {
        return drn.dump(m_array_body);
    }
    const ArrayBody &array_body() const { return m_array_body; }
    void clear() { m_array_body.clear(); }
    template <class Policy_>
    Failable<Walker> find(const CharType *k, SizeType kl) const
    {
        Walker w(*this, k, kl);
        if (Status rv = w.find<Policy_>())
            return rv;
        return w;
    }
    //
    ~DoubleArray() = default;
    DoubleArray() = default;
private:
    ArrayBody m_array_body{};
};

template <class Traits_>
class DoubleArray<Traits_>::Walker
{
public:
    class ExactPolicy;
    class MostCommonPolicy;
    class LeastCommonPolicy;
    ~Walker() = default;
    Walker() = default;
    Walker(const DoubleArray &da, const CharType *k, SizeType kl)
        : m_array_body{&da.m_array_body},
          m_key{k}, m_key_length{kl},
          m_id{m_array_body->base(0, 0)}
    {
    }
    Walker(const Walker &w, const CharType *subkey, SizeType skl)
        : m_array_body{w.m_array_body},
          m_key{subkey}, m_key_length{skl},
          m_id{w.m_id}, m_depth{0}
    {
    }
    template <class CommonPrefixCallback_>
    Status operator () (CommonPrefixCallback_ cb)
    {
        if (m_depth > m_key_length)
            return S_BREAK;

        if (auto rv = this->is_leaf() ? cb(*this) : S_OK)
            return rv;

        const auto ofs =
            m_depth == m_key_length ?
            Traits_::TERMINATOR : Traits_::char_to_node_offset(m_key[m_depth]);

        if (!m_array_body->is_check_ok(m_id, ofs))
            return S_NO_ENTRY;

        m_id = m_array_body->base(m_id, ofs);
        m_depth++;

        return ofs == Traits_::TERMINATOR ? S_BREAK : S_OK;
    }
    template <class Policy_> Status find() { return Policy_::find(*this); }
    Status find_exact()
    { return this->template find<ExactPolicy>(); }
    Status find_most_common()
    { return this->template find<MostCommonPolicy>(); }
    Status find_least_common()
    { return this->template find<LeastCommonPolicy>(); }
    NodeIDType id() const { return m_id; }
    const CharType *key() const { return m_key; }
    SizeType key_length() const { return m_key_length; }
    SizeType depth() const { return std::min(m_depth, m_key_length); }
    bool is_valid() const { return m_array_body != nullptr; }
    bool is_done() const { return m_depth > m_key_length; }
    bool is_leaf() const
    { return m_array_body->is_check_ok(m_id, Traits_::TERMINATOR); }
    NodeIDType get_leaf_id() const
    {
        AMDA_ASSERT(this->is_leaf());
        return m_array_body->base(m_id, Traits_::TERMINATOR);
    }
private:
    const ArrayBody *m_array_body = nullptr;
    const CharType *m_key = nullptr;
    SizeType m_key_length = 0;
    NodeIDType m_id = 0;
    SizeType m_depth = 0;
};

template <class Traits_>
class DoubleArray<Traits_>::Walker::ExactPolicy
{
public:
    static Status find(Walker &w)
    {
        Status rv;

        while (!(rv = w([](auto) { return S_OK; })))
            ;

        if (rv == S_BREAK)
            rv = S_OK;

        return rv;
    }
};


template <class Traits_>
class DoubleArray<Traits_>::Walker::MostCommonPolicy
{
public:
    static Status find(Walker &w)
    {
        Status rv;
        Walker saved;

        while (!(rv = w([&saved](const auto &w) { saved=w; return S_OK; })))
            ;

        switch (rv) {
        case S_BREAK:
            return S_OK;
        case S_NO_ENTRY:
            if (!saved.is_valid())
                return S_NO_ENTRY;
            w = saved;
            return S_OK;
        }
        return rv;
    }
};

template <class Traits_>
class DoubleArray<Traits_>::Walker::LeastCommonPolicy
{
public:
    static Status find(Walker &w)
    {
        Status rv;

        while (!(rv = w([](const auto &) { return S_BREAK; })))
            ;

        if (rv == S_BREAK)
            rv = S_OK;

        return rv;
    }
};


// ----------------------------------------------------------------------

template <class Traits_, class Source_>
class ScratchBuilder : NonCopyable, NonMovable
{
public:
    using ArrayBody = typename Traits_::ArrayBody;
    using SizeType = typename Traits_::SizeType;
    using CharType = typename Traits_::CharType;
    using NodeIDType = typename Traits_::NodeIDType;
    static Failable<ArrayBody> build(const Source_ &src)
    {
        return ScratchBuilder{src}.build_();
    }
private:
    using ArrayBodyFactory_ = typename Traits_::ArrayBody::ScratchFactory;
    // Trie:
    //   - a kind of state machine.
    //   - a node indicates a state.
    //     the root is the initial state.
    //   - a edge is corresponding to each char of key.
    // Note: To build a double array, only the subtree of the trie
    //       is necessary because of the depth-first insertion.

    // node.
    class Node_
    {
    public:
        ~Node_() = default;
        // uninitialized node.
        Node_() = default;
        // node having edges between [l, r).
        Node_(SizeType l, SizeType r) : m_left{l}, m_right{r} { }
        // root node
        explicit Node_(const Source_ &s)
            : m_left{0}, m_right{s.num_entries()} { }
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
        void set(SizeType l, SizeType r)
        { m_left = l; m_right = r; }
        //
        SizeType norm() const { return m_right - m_left; }
    private:
        SizeType m_left;
        SizeType m_right;
    };
    // edge.
    class Edge_
    {
    public:
        ~Edge_() = default;
        Edge_() = default;
        Edge_(SizeType l, SizeType r, SizeType c) : m_node{l, r}, m_code{c} { }
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
    Failable<EdgeQueue_> fetch_edges_(const Node_ &parent,
                                      SizeType parent_depth) const
    {
        EdgeQueue_ q;
        AMDA_ASSERT(parent.norm() > 0);

        Edge_ *last_edge = nullptr;
        auto char_of_edge = Traits_::TERMINATOR;
        auto prev_char = Traits_::TERMINATOR;

        for (SizeType i=parent.left(); i<parent.right(); i++) {
            const auto keylen = m_source.key_length(i);
            if (last_edge == nullptr && keylen == parent_depth) {
                AMDA_ASSERT(char_of_edge == Traits_::TERMINATOR);
                AMDA_ASSERT(prev_char == Traits_::TERMINATOR);
                AMDA_ASSERT(i==parent.left());
                // leaf.
            } else if (keylen <= parent_depth) {
                // not sorted nor unique.
                return S_INVALID_KEY;
            } else {
                // node.
                char_of_edge =
                    Traits_::char_to_node_offset(
                        m_source.key(i)[parent_depth]);
            }
            if (last_edge == nullptr ||
                char_of_edge != prev_char) {
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
    SizeType fit_edges_(const EdgeQueue_ &q)
    {
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
            if (m_array_factory.is_inuse(pos)) {
                num_filled++;
                goto retry;
            }
            if (first) {
                first = false;
                m_next_check_pos = pos;
            }
            node = pos - first_code;
            if (m_used_node_id_mask.size() <= node)
                m_used_node_id_mask.resize(node+1);
            if (m_used_node_id_mask[node]) {
                // node ID must be unique.
                // don't confuse in-used element of the array.
                goto retry;
            }
            m_array_factory.expand(node+last_code+1);
            // determine whether the all chars fit to the holes.
            for (const auto e : make_iter_view(q.begin()+1, q.end())) {
                if (m_array_factory.is_inuse(node+e.code()))
                    goto retry;
            }
            // now, node ID is determined.
            break;
retry:
            pos++;
        }
        if (Traits_::is_too_dense(num_filled, pos-m_next_check_pos+1)){
            // there are few spaces.  skip the block.
            m_next_check_pos = pos;
        }

        return node;
    }
    //
    Status insert_edges_(SizeType node_id, const EdgeQueue_ &q,
                         SizeType parent_depth)
    {

        AMDA_ASSERT(q.size() > 0);
        AMDA_ASSERT(m_used_node_id_mask.size() > node_id);
        AMDA_ASSERT(m_used_node_id_mask[node_id] == false);

        m_used_node_id_mask[node_id] = true;

        // ensure to reserve the elements before recursive call.
        for (const auto &e : q)
            m_array_factory.set_inuse(node_id, e.code());

        // insert descendants.
        for (const auto &e : q) {
            if (e.code() == Traits_::TERMINATOR) {
                // base[id + ch] expresses the edge to
                // the leaf node.
                AMDA_ASSERT(e.node().norm() == 1);
                AMDA_ASSERT(&e==&*q.begin());
                m_array_factory.set_base(
                    node_id, Traits_::TERMINATOR,
                    m_source.get_leaf_id(e.node().left()));
            } else {
                // base[id + ch]  expresses the edge to
                // the other node.
                auto fcid = insert_children_(e.node(), parent_depth+1);
                if (!fcid.apply([&](auto cid) {
                            m_array_factory.set_base(node_id, e.code(), cid);
                            return 0; // dummy to propagate success.
                        }))
                    return fcid;
            }
        }
        return S_OK;
    }
    Failable<SizeType> insert_children_(const Node_ &parent,
                                        SizeType parent_depth)
    {
        auto fq = fetch_edges_(parent, parent_depth);

        if (fq) {
            auto q = fq.unwrap();
            auto node_id = fit_edges_(q);
            if (auto rv = insert_edges_(node_id, q, parent_depth))
                return rv;

            return node_id;
        }
        return fq;
    }
    Failable<ArrayBody> build_()
    {
        if (m_source.num_entries() == 0)
            return S_NO_ENTRY;

        m_next_check_pos = 1;
        m_used_node_id_mask.resize(1);
        m_used_node_id_mask[0] = true;

        m_array_factory.start();

        return insert_children_(Node_{m_source}, 0).apply([&](auto node_id) {
                // set base[0] to the root node ID, which should be 1.
                // note: node #0 is not a valid node.
                AMDA_ASSERT(node_id != 0);
                m_array_factory.set_base(0, Traits_::TERMINATOR, node_id);
                return m_array_factory.done();
            });
    }
    ScratchBuilder(const Source_ &src) : m_source{src} { }
    ~ScratchBuilder() = default;
    //
    const Source_ &m_source;
    ArrayBodyFactory_ m_array_factory;
    SizeType m_next_check_pos = 0;
    // already used node IDs.
    UsedNodeIdMask_ m_used_node_id_mask;
};


// ----------------------------------------------------------------------

template <class Traits_>
class PersistBuilder
{
public:
    using ArrayBody = typename Traits_::ArrayBody;
    using SizeType = typename Traits_::SizeType;
    using NodeIDType = typename Traits_::NodeIDType;
private:
    using ArrayBodyFactory_ = typename Traits_::ArrayBody::PersistFactory;
public:
    template <class Source_>
    static Failable<ArrayBody> build(const Source_ &src)
    {
        return ArrayBodyFactory_{}.load(src);
    }
};

// ----------------------------------------------------------------------
// Standard implementation for normal usage.
//

namespace Standard
{

// ----------------------------------------------------------------------
// standard ArrayBody
//

template <class Traits_>
class ArrayBody : NonCopyable
{
public:
    using SizeType = typename Traits_::SizeType;
    using NodeIDType = typename Traits_::NodeIDType;
    using Storage = typename Traits_::Storage;
    class ScratchFactory;
    class PersistFactory;
public:
    ~ArrayBody() = default;
    // interface for class DoubleArray.
    SizeType num_entries() const { return m_storage.num_entries(); }
    bool is_inuse(NodeIDType nid, SizeType ofs) const
    {
        ofs += nid;
        return m_storage.is_inuse(ofs);
    }
    NodeIDType check(NodeIDType nid, SizeType ofs) const
    {
        AMDA_ASSERT((SizeType)nid+ofs < m_storage.num_entries());
        return m_storage.check((SizeType)nid+ofs);
    }
    bool is_check_ok(NodeIDType nid, SizeType ofs) const
    {
        ofs += nid;
        return m_storage.is_inuse(ofs) && m_storage.check(ofs) == nid;
    }
    NodeIDType base(NodeIDType nid, SizeType ofs) const
    {
        AMDA_ASSERT((SizeType)nid+ofs < m_storage.num_entries());
        return m_storage.base((SizeType)nid+ofs);
    }
    void clear()
    {
        m_storage.reset(nullptr);
    }
    ArrayBody() = default;
    ArrayBody(Storage &&s) : m_storage(std::move(s)) { }
    ArrayBody(ArrayBody &&) = default;
    ArrayBody &operator = (ArrayBody &&) = default;
    ArrayBody &operator = (Storage &&s)
    {
        m_storage = std::move(s);
        return *this;
    }
    const Storage &storage() const { return m_storage; }
private:
    Storage m_storage;
};

template <class Traits_>
class ArrayBody<Traits_>::ScratchFactory : NonCopyable, NonMovable
{
private:
    using StorageFactory_ = typename Storage::ScratchFactory;
public:
    ~ScratchFactory() = default;
    ScratchFactory() = default;
    void expand(SizeType sz) { m_storage_factory.expand(sz); }
    bool is_inuse(SizeType idx) const
    {
        return
            idx < m_storage_factory.num_entries() &&
                  m_storage_factory.is_inuse(idx);
    }
    void set_inuse(NodeIDType nid, SizeType ofs)
    {
        ofs += (SizeType)nid;
        AMDA_ASSERT(!m_storage_factory.is_inuse(ofs));
        m_storage_factory.expand(ofs+1);
        m_storage_factory.set_inuse(ofs);
        m_storage_factory.set_check(ofs, nid);
    }
    void set_base(NodeIDType base, SizeType ofs, NodeIDType nid)
    {
        ofs += (SizeType)base;
        this->expand(ofs+1);
        m_storage_factory.set_base(ofs, nid);
    }
    void start() { m_storage_factory.start(); }
    ArrayBody done()
    {
        return m_storage_factory.done();
    }
private:
    StorageFactory_ m_storage_factory;
};


// ----------------------------------------------------------------------

template <class Traits_>
class ArrayBody<Traits_>::PersistFactory
{
public:
    template <class Source_>
    Failable<ArrayBody> load(const Source_ &src)
    {
        return src.load();
    }
};


// ----------------------------------------------------------------------

template <class Traits_>
class SortedKeySource
{
public:
    using CharType = typename Traits_::CharType;
    using SizeType = typename Traits_::SizeType;
    using NodeIDType = typename Traits_::NodeIDType;
    using Builder = ScratchBuilder<Traits_, SortedKeySource>;
private:
    using KeyType_ = const CharType *;
public:
    ~SortedKeySource() = default;
    SortedKeySource(SizeType ne, const KeyType_ *k, const SizeType *kl,
                    const NodeIDType *lid = nullptr)
        : m_num_entries{ne}, m_keys{k}, m_key_lengths{kl}, m_leaf_ids{lid}
    {
    }
    SizeType num_entries() const { return m_num_entries; }
    NodeIDType get_leaf_id(SizeType idx) const
    { return m_leaf_ids ? m_leaf_ids[idx] : (NodeIDType)idx; }
    KeyType_ key(SizeType idx) const { return m_keys[idx]; }
    SizeType key_length(SizeType idx) const
    { return m_key_lengths[idx]; }
private:
    SizeType m_num_entries;
    const KeyType_ *m_keys;
    const SizeType *m_key_lengths;
    const NodeIDType *m_leaf_ids;
};


// ----------------------------------------------------------------------

template <class, class> class FileAccessorTmpl;

template <class Traits_>
class FileSource
{
private:
    using ArrayBody_ = typename Traits_::ArrayBody;
    using Storage_ = typename Traits_::Storage;
    using Accessor_ = FileAccessorTmpl<Traits_, Storage_>;
public:
    using Builder = PersistBuilder<Traits_>;
    //
    ~FileSource() = default;
    FileSource(const std::string &fn) : m_filename{fn} { }
    Failable<ArrayBody_> load() const
    {
        return Accessor_::load(m_filename);
    }
private:
    std::string m_filename;
};


template <class Traits_>
class FileDrain
{
private:
    using ArrayBody_ = typename Traits_::ArrayBody;
    using Storage_ = typename Traits_::Storage;
    using Accessor_ = FileAccessorTmpl<Traits_, Storage_>;
public:
    ~FileDrain() = default;
    FileDrain(const std::string &fn) : m_filename{fn} { }
    Status dump(const ArrayBody_ &body) const
    {
        return Accessor_::save(m_filename, body);
    }
private:
    std::string m_filename;
};


// ----------------------------------------------------------------------
// storage type which separates base and check into individual arrays.
//

template <class Traits_>
class SeparatedStorage : NonCopyable
{
public:
    using SizeType = typename Traits_::SizeType;
    using NodeIDType = typename Traits_::NodeIDType;
    using BaseType = typename Traits_::BaseType;
    using CheckType = typename Traits_::CheckType;
    class ScratchFactory;
    class HouseKeeper
    {
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
    SeparatedStorage &operator = (SeparatedStorage &&) = default;
    explicit SeparatedStorage(HouseKeeper *hk) { reset(hk); }
    void reset(HouseKeeper *hk = nullptr)
    {
        m_house_keeper.reset(hk);
        if (hk) {
            m_num_entries = hk->num_entries();
            m_bases = hk->bases();
            m_checks = hk->checks();
        }
    }
    SizeType num_entries() const { return m_num_entries; }
    NodeIDType base(SizeType idx) const
    { return Traits_::base_to_nid(idx, m_bases[idx]); }
    NodeIDType check(SizeType idx) const
    { return Traits_::check_to_nid(idx, m_checks[idx]); }
    bool is_inuse(SizeType idx) const
    { return Traits_::is_inuse(m_bases[idx], m_checks[idx]); }
    const HouseKeeper &house_keeper() const { return *m_house_keeper; }
private:
    std::unique_ptr<HouseKeeper> m_house_keeper;
    SizeType m_num_entries = 0;
    const BaseType *m_bases = nullptr;
    const CheckType *m_checks = nullptr;
};
template <class Traits_>
SeparatedStorage<Traits_>::HouseKeeper::~HouseKeeper() { }

template <class Traits_>
class SeparatedStorage<Traits_>::ScratchFactory : NonCopyable, NonMovable
{
private:
    using BaseArray_ = std::vector<BaseType>;
    using CheckArray_ = std::vector<CheckType>;
    class VariableSizedHouseKeeper_ final : public HouseKeeper
    {
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
    // for ScratchFactory interface
    SizeType num_entries() const { return this->num_entries_(); }
    void set_base(SizeType idx, NodeIDType nid)
    { this->bases_()[idx] = Traits_::nid_to_base(idx, nid); }
    NodeIDType base(SizeType idx) const
    { return Traits_::base_to_nid(idx, this->bases_()[idx]); }
    void set_check(SizeType idx, NodeIDType nid)
    { this->checks_()[idx] = Traits_::nid_to_check(idx, nid); }
    NodeIDType check(SizeType idx) const
    { return Traits_::check_to_nid(idx, this->checks_()[idx]); }
    void set_inuse(SizeType idx)
    {
        Traits_::set_inuse(idx,
                           &this->bases_()[idx],
                           &this->checks_()[idx]);
    }
    bool is_inuse(SizeType idx) const
    {
        return Traits_::is_inuse(this->bases_()[idx],
                                 this->checks_()[idx]);
    }
    void expand(SizeType s)
    {
        if (s > this->num_entries_()) {
            this->bases_().resize(s, 0);
            this->checks_().resize(s, 0);
            this->num_entries_() = s;
        }
    }
    void start()
    {
        m_house_keeper.reset(new VariableSizedHouseKeeper_);
    }
    SeparatedStorage done()
    {
        return SeparatedStorage{m_house_keeper.release()};
    }
    //
    ~ScratchFactory() = default;
    ScratchFactory() = default;
private:
    std::unique_ptr<VariableSizedHouseKeeper_> m_house_keeper;
};


template <class Traits_>
class FileAccessorTmpl<Traits_, SeparatedStorage<Traits_> >
{
public:
    using SizeType = typename Traits_::SizeType;
    using BaseType = typename Traits_::BaseType;
    using CheckType = typename Traits_::CheckType;
private:
    using ArrayBody_ = typename Traits_::ArrayBody;
    using Storage_ = SeparatedStorage<Traits_>;
    using HouseKeeper_ = typename Storage_::HouseKeeper;
    class FixedSizedHouseKeeper_ final : public HouseKeeper_
    {
public:
        // for HouseKeeper interface
        ~FixedSizedHouseKeeper_() override = default;
        SizeType num_entries() const override { return m_num_entries; }
        const BaseType *bases() const override { return m_bases.get(); }
        const CheckType *checks() const override { return m_checks.get(); }
        //
        FixedSizedHouseKeeper_(SizeType n)
            : m_num_entries{n},
              m_bases{new BaseType [n]},
              m_checks{new CheckType [n]}
        {
        }
        BaseType *bases() { return m_bases.get(); }
        CheckType *checks() { return m_checks.get(); }
private:
        SizeType m_num_entries;
        std::unique_ptr<BaseType[]> m_bases;
        std::unique_ptr<CheckType[]> m_checks;
    };
public:
    // XXX: machine dependent.
    static Status save(const std::string &fn, const ArrayBody_ &body)
    {
        std::unique_ptr<std::FILE, decltype (&fclose)> fp(
            std::fopen(fn.c_str(), "wb"), &std::fclose);

        if (!fp)
            return S_IO_ERROR;

        std::unique_ptr<const std::string, Unlinker> unlinker(&fn);
        const HouseKeeper_ &hk = body.storage().house_keeper();
        SizeType ne = hk.num_entries();

        if (std::fwrite(&ne, sizeof (SizeType), 1, fp.get()) != 1)
            return S_IO_ERROR;
        if (std::fwrite(hk.bases(), sizeof (BaseType), ne,
                        fp.get()) != ne)
            return S_IO_ERROR;
        if (std::fwrite(hk.checks(), sizeof (CheckType), ne,
                        fp.get()) != ne)
            return S_IO_ERROR;

        unlinker.release();

        return S_OK;
    }
    // XXX: machine dependent.
    static Failable<ArrayBody_> load(const std::string &fn)
    {

        std::unique_ptr<std::FILE, decltype (&std::fclose)> fp(
            std::fopen(fn.c_str(), "rb"), &std::fclose);

        if (!fp)
            return S_NO_ENTRY;

        SizeType ne;
        if (std::fread(&ne, sizeof (ne), 1, fp.get()) != 1 || ne == 0)
            return S_INVALID_DATA;

        std::unique_ptr<FixedSizedHouseKeeper_> hk(
            new FixedSizedHouseKeeper_(ne));

        if (std::fread(hk->bases(), sizeof (BaseType), ne,
                       fp.get()) != ne)
            return S_INVALID_DATA;
        if (std::fread(hk->checks(), sizeof (CheckType), ne,
                       fp.get()) != ne)
            return S_INVALID_DATA;

        return ArrayBody_{Storage_{hk.release()}};
    }
};


// ----------------------------------------------------------------------
// storage type which stores base and check into a structure of
// the array element.
//

template <class Traits_>
class StructuredStorage : NonCopyable
{
public:
    using SizeType = typename Traits_::SizeType;
    using NodeIDType = typename Traits_::NodeIDType;
    using BaseType = typename Traits_::BaseType;
    using CheckType = typename Traits_::CheckType;
    using ElementType = typename Traits_::ElementType;
    class ScratchFactory;
    class HouseKeeper
    {
    public:
        virtual ~HouseKeeper() = 0;
        virtual SizeType num_entries() const = 0;
        virtual const ElementType *elements() const = 0;
    };
    //
    ~StructuredStorage() = default;
    StructuredStorage() = default;
    StructuredStorage(StructuredStorage &&) = default;
    StructuredStorage &operator = (StructuredStorage &&) = default;
    explicit StructuredStorage(HouseKeeper *hk) { reset(hk); }
    void reset(HouseKeeper *hk = nullptr)
    {
        m_house_keeper.reset(hk);
        if (hk) {
            m_num_entries = hk->num_entries();
            m_elements = hk->elements();
        }
    }
    SizeType num_entries() const { return m_num_entries; }
    NodeIDType base(SizeType idx) const
    { return Traits_::base_to_nid(idx, m_elements[idx].base); }
    NodeIDType check(SizeType idx) const
    { return Traits_::check_to_nid(idx, m_elements[idx].check); }
    bool is_inuse(SizeType idx) const
    {
        return Traits_::is_inuse(m_elements[idx].base,
                                 m_elements[idx].check);
    }
    const HouseKeeper &house_keeper() const { return *m_house_keeper; }
private:
    std::unique_ptr<HouseKeeper> m_house_keeper;
    SizeType m_num_entries = 0;
    const ElementType *m_elements = nullptr;
};
template <class Traits_>
StructuredStorage<Traits_>::HouseKeeper::~HouseKeeper() { }

template <class Traits_>
class StructuredStorage<Traits_>::ScratchFactory : NonCopyable, NonMovable
{
private:
    using ElementArray_ = std::vector<ElementType>;
    class VariableSizedHouseKeeper_ final : public HouseKeeper
    {
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
    const ElementArray_ &elements_() const
    { return m_house_keeper->m_elements; }
public:
    // for ScratchFactory interface
    SizeType num_entries() const { return this->num_entries_(); }
    void set_base(SizeType idx, NodeIDType nid)
    { this->elements_()[idx].base = Traits_::nid_to_base(idx, nid); }
    NodeIDType base(SizeType idx) const
    { return Traits_::base_to_nid(idx, this->elements_()[idx].base); }
    void set_check(SizeType idx, NodeIDType nid)
    { this->elements_()[idx].check = Traits_::nid_to_check(idx, nid); }
    NodeIDType check(SizeType idx) const
    { return Traits_::check_to_nid(idx, this->elements_()[idx].check); }
    void set_inuse(SizeType idx)
    {
        Traits_::set_inuse(idx,
                           &this->elements_()[idx].base,
                           &this->elements_()[idx].check);
    }
    bool is_inuse(SizeType idx) const
    {
        return Traits_::is_inuse(this->elements_()[idx].base,
                                 this->elements_()[idx].check);
    }
    void expand(SizeType s)
    {
        if (s > this->num_entries_()) {
            static const ElementType z = { 0, 0 };
            this->elements_().resize(s, z);
            this->num_entries_() = s;
        }
    }
    void start()
    {
        m_house_keeper.reset(new VariableSizedHouseKeeper_);
    }
    StructuredStorage done()
    {
        return StructuredStorage{m_house_keeper.release()};
    }
    //
    ~ScratchFactory() = default;
    ScratchFactory() = default;
private:
    std::unique_ptr<VariableSizedHouseKeeper_> m_house_keeper;
};


template <class Traits_>
class FileAccessorTmpl<Traits_, StructuredStorage<Traits_> >
{
public:
    using SizeType = typename Traits_::SizeType;
    using ElementType = typename Traits_::ElementType;
private:
    using ArrayBody_ = typename Traits_::ArrayBody;
    using Storage_ = StructuredStorage<Traits_>;
    using HouseKeeper_ = typename Storage_::HouseKeeper;
    class FixedSizedHouseKeeper_ final : public HouseKeeper_
    {
public:
        // for HouseKeeper interface
        ~FixedSizedHouseKeeper_() override = default;
        SizeType num_entries() const override { return m_num_entries; }
        const ElementType *elements() const override
        { return m_elements.get(); }
        //
        FixedSizedHouseKeeper_(SizeType n)
            : m_num_entries{n}, m_elements{new ElementType [n]}
        {
        }
        ElementType *elements() { return m_elements.get(); }
private:
        SizeType m_num_entries;
        std::unique_ptr<ElementType[]> m_elements;
    };
public:
    // XXX: machine dependent.
    static Status save(const std::string &fn, const ArrayBody_ &body)
    {
        std::unique_ptr<std::FILE, decltype (&std::fclose)> fp(
            std::fopen(fn.c_str(), "wb"), &std::fclose);

        if (!fp)
            return S_IO_ERROR;

        std::unique_ptr<const std::string, Unlinker> unlinker(&fn);

        const HouseKeeper_ &hk = body.storage().house_keeper();
        SizeType ne = hk.num_entries();

        if (std::fwrite(&ne, sizeof (SizeType), 1, fp.get()) != 1)
            return S_IO_ERROR;
        if (std::fwrite(hk.elements(), sizeof (ElementType), ne,
                        fp.get()) != ne)
            return S_IO_ERROR;

        unlinker.release();

        return S_OK;
    }
    // XXX: machine dependent.
    static Failable<ArrayBody_> load(const std::string &fn)
    {

        std::unique_ptr<std::FILE, decltype (&std::fclose)> fp(
            std::fopen(fn.c_str(), "rb"), &std::fclose);

        if (!fp)
            return S_IO_ERROR;

        SizeType ne;
        if (std::fread(&ne, sizeof (ne), 1, fp.get()) != 1 || ne == 0)
            return S_IO_ERROR;

        std::unique_ptr<FixedSizedHouseKeeper_> hk(
            new FixedSizedHouseKeeper_(ne));

        if (std::fread(hk->elements(), sizeof (ElementType), ne,
                       fp.get()) != ne)
            return S_IO_ERROR;

        return ArrayBody_{Storage_{hk.release()}};
    }
};

// ----------------------------------------------------------------------

template <typename BaseType_, typename CheckType_>
struct Element
{
    BaseType_ base;
    CheckType_ check;
};


// ----------------------------------------------------------------------

template <typename CharType_, typename SizeType_, typename NodeIDType_,
          template <class> class StorageType_,
          typename BaseType_=NodeIDType_, typename CheckType_=NodeIDType_,
          class ElementType_ = Element<BaseType_, CheckType_> >
struct Traits
{
    using CharType = CharType_;
    using SizeType = SizeType_;
    using NodeIDType = NodeIDType_;
    using ArrayBody = ArrayBody<Traits>;
    using Storage = StorageType_<Traits>;
    // the node offset for the terminator.
    static constexpr SizeType_ TERMINATOR = 0;
    // convert character code to corresponding node offset.
    static SizeType_ char_to_node_offset(CharType ch)
    { return (SizeType_)ch+1; }
    // whether the zone is too dense.
    static bool is_too_dense(SizeType num_filled, SizeType extent)
    { return (float)num_filled / extent >= 0.95; }
    // for Storage
    using BaseType = BaseType_;
    using CheckType = CheckType_;
    using ElementType = ElementType_; // only for StructuredStorage
    static NodeIDType_ base_to_nid(SizeType_, BaseType_ v)
    { return NodeIDType(v); }
    static BaseType_ nid_to_base(SizeType_, NodeIDType_ v)
    { return BaseType(v); }
    static NodeIDType_ check_to_nid(SizeType_, CheckType_ v)
    { return NodeIDType(v); }
    static CheckType_ nid_to_check(SizeType_, NodeIDType_ v)
    { return CheckType(v); }
    static void set_inuse(SizeType_, BaseType_ *, CheckType_ *)
    {
        // it is done by setting to check array.
    }
    static bool is_inuse(BaseType_, CheckType_ chk)
    { return chk != 0; }
};

} // Standard


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

namespace DeltaCheck
{

using Standard::ArrayBody;
using Standard::SortedKeySource;
using Standard::FileSource;
using Standard::FileDrain;
using Standard::SeparatedStorage;

template <typename SizeType_, typename NodeIDType_,
          typename BaseType_=NodeIDType_>
struct Traits
{
    using CharType = U8;
    using SizeType = SizeType_;
    using NodeIDType = NodeIDType_;
    using ArrayBody = ArrayBody<Traits>;
    using Storage = SeparatedStorage<Traits>;
    static constexpr SizeType_ TERMINATOR = 0;
    static SizeType_ char_to_node_offset(CharType ch)
    { return (SizeType_)ch+1; }
    static bool is_too_dense(SizeType num_filled, SizeType extent)
    { return (float)num_filled / extent >= 0.95; }
    using BaseType = BaseType_;
    using CheckType = U8;
    static NodeIDType_ base_to_nid(SizeType_, BaseType_ v)
    { AMDA_ASSERT(v!=0); return NodeIDType_(v)-1; }
    static BaseType_ nid_to_base(SizeType_, NodeIDType_ v)
    { return BaseType_(v+1); }
    static NodeIDType_ check_to_nid(SizeType_ idx, CheckType v)
    { return NodeIDType_(idx - SizeType_(v)); }
    static CheckType nid_to_check(SizeType_ idx, NodeIDType_ v)
    { return U8(idx - SizeType_(v)); }
    static void set_inuse(SizeType_, BaseType_ *pb, CheckType *)
    {
        *pb = 1;  // dummy to indicate in-used.
    }
    static bool is_inuse(BaseType_ b, CheckType)
    { return b != 0; }
};


} // DeltaCheck

} // AMDA

#endif
