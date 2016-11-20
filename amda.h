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

//
// static double array.
//

#ifndef AM_DOUBLE_ARRAY_H_
#define AM_DOUBLE_ARRAY_H_

#include <vector>
#include <string>
#include <memory>
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
};

template <typename T_>
inline T_ disown(T_ *val, T_ n=T_()) { T_ old=*val; *val=n; return old; }

template <typename T_>
inline void swap(T_ *v1, T_ *v2) { *v1 = disown(v2, *v1); }

template <typename T_>
inline T_ max(T_ a, T_ b) { return a>b ? a:b; }

template <typename T_>
inline T_ min(T_ a, T_ b) { return a<b ? a:b; }

struct Unlinker
{
	void operator () (const std::string *file) const
	{
		if (file) unlink(file->c_str());
	}
};

// XXX
typedef char S8;
typedef unsigned char U8;

#ifdef AMDA_DEBUG
#define AMDA_ASSERT(cond)						      \
do {									      \
	if (/*CONSTCOND*/(cond))					      \
		;							      \
	else {								      \
		std::fprintf(stderr,					      \
			     "assertion failed at %s:%d : \n\t%s\n",	      \
			     __FILE__, __LINE__, #cond);		      \
		std::abort();						      \
	}								      \
} while (/*CONSTCOND*/0)
#else
#define AMDA_ASSERT(cond)
#endif

#define AMDA_NOCOPY(cls)						      \
private:								      \
	void operator = (const cls &) = delete;				      \
	cls(const cls &) = delete


// ----------------------------------------------------------------------

template <class Traits_>
class DoubleArray
{
	AMDA_NOCOPY(DoubleArray);
public:
	typedef typename Traits_::ArrayBody ArrayBody;
	typedef typename Traits_::SizeType SizeType;
	typedef typename Traits_::CharType CharType;
	typedef typename Traits_::NodeIDType NodeIDType;
	//
	class Walker;
	//
	template <class Source_>
	Status build(const Source_ &src)
	{
		typename Source_::Factory f;
		this->clear();
		return f.build(&m_array_body, src);
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
	void clear() { m_array_body.reset(); }
	//
	~DoubleArray() = default;
	DoubleArray() = default;
private:
	ArrayBody m_array_body{};
};

template <class Traits_>
class DoubleArray<Traits_>::Walker
{
	// copyable.
private:
	template <class CommonPrefixCallback_, class Wrapper_>
	Status walk_(CommonPrefixCallback_ &cb)
	{
		if (m_depth > m_key_length)
			return S_BREAK;

		{
			Status rv;
			if ((rv = Wrapper_::call_if_leaf(*this, cb)))
				return rv;
		}

		const SizeType term = Traits_::get_terminator();
		const NodeIDType ofs =
		    m_depth == m_key_length ?
		    term : Traits_::char_to_node_offset(m_key[m_depth]);

		if (!m_array_body->is_check_ok(m_id, ofs))
			return S_NO_ENTRY;

		m_id = m_array_body->base(m_id, ofs);
		m_depth++;

		return ofs == term ? S_BREAK : S_OK;
	}
	template <class CommonPrefixCallback_>
	struct CPCWrapper_
	{
		static Status call_if_leaf(const Walker &w,
					   CommonPrefixCallback_ &cb)
		{
			Status rv = S_OK;
			if (w.is_leaf())
				rv = cb(w);
			return rv;
		}
	};
	struct CPCNullWrapper_
	{
		static Status call_if_leaf(const Walker &,
					   const int &)
		{ return S_OK; }
	};
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
		  m_id{w.m_id}, m_depth{w.is_done() ? skl+1:0}
	{
	}
	Status operator () ()
	{
		return this->template walk_<const int, CPCNullWrapper_>(0);
	}
	template <class CommonPrefixCallback_>
	Status operator () (CommonPrefixCallback_ &closure)
	{
		return this->template walk_<CommonPrefixCallback_,
		    CPCWrapper_<CommonPrefixCallback_> >(closure);
	}
	template <class CommonPrefixCallback_>
	Status operator () (const CommonPrefixCallback_ &closure)
	{
		return this->template walk_<const CommonPrefixCallback_,
		    CPCWrapper_<const CommonPrefixCallback_> >(closure);
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
	SizeType depth() const { return min(m_depth, m_key_length); }
	bool is_valid() const { return m_array_body != nullptr; }
	bool is_done() const { return m_depth > m_key_length; }
	bool is_leaf() const
	{ return m_array_body->is_check_ok(m_id, Traits_::get_terminator()); }
	NodeIDType get_leaf_id() const
	{
		AMDA_ASSERT(this->is_leaf());
		return m_array_body->base(m_id, Traits_::get_terminator());
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

		while (!(rv = w()))
			;

		if (rv == S_BREAK)
			rv = S_OK;

		return rv;
	}
};


template <class Traits_>
class DoubleArray<Traits_>::Walker::MostCommonPolicy
{
private:
	class Callback_
	{
		Walker m_saved{};
	public:
		~Callback_() = default;
		Callback_() = default;
		Status operator () (const Walker &w)
		{
			m_saved = w;
			return S_OK;
		}
		const Walker &saved() const { return m_saved; }
	};
public:
	static Status find(Walker &w)
	{
		Status rv;
		Callback_ cb;

		while (!(rv = w(cb)))
			;

		switch (rv) {
		case S_BREAK:
			return S_OK;
		case S_NO_ENTRY:
			if (!cb.saved().is_valid())
				return S_NO_ENTRY;
			w = cb.saved();
			return S_OK;
		}
		return rv;
	}
};

template <class Traits_>
class DoubleArray<Traits_>::Walker::LeastCommonPolicy
{
private:
	struct Callback_
	{
		Status operator () (const Walker &) const { return S_BREAK; }
	};
public:
	static Status find(Walker &w)
	{
		Status rv;

		while (!(rv = w(Callback_())))
			;

		if (rv == S_BREAK)
			rv = S_OK;

		return rv;
	}
};


// ----------------------------------------------------------------------

template <class Traits_, class Source_>
class ScratchFactory
{
	AMDA_NOCOPY(ScratchFactory);
public:
	typedef typename Traits_::ArrayBody ArrayBody;
	typedef typename Traits_::SizeType SizeType;
	typedef typename Traits_::CharType CharType;
	typedef typename Traits_::NodeIDType NodeIDType;
private:
	typedef typename Traits_::ArrayBody::ScratchFactory ArrayBodyFactory;
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
		Node_(SizeType l, SizeType r)
			: m_left(l), m_right(r) { }
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
		void set_to_root(const Source_ &s)
		{ m_left = 0; m_right = s.num_entries(); }
		SizeType norm() const { return m_right - m_left; }
	private:
		SizeType m_left;
		SizeType m_right;
	};
	class RootNode_ : public Node_
	{
	public:
		~RootNode_() = default;
		RootNode_(const Source_ &s) { this->Node_::set_to_root(s); }
	};
	// edge.
	class Edge_
	{
	public:
		~Edge_() = default;
		Edge_() = default;
		Edge_(SizeType l, SizeType r, SizeType c)
			: m_node(l, r), m_code(c)
		{ }
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
	typedef std::vector<Edge_> EdgeQueue_;
	typedef typename EdgeQueue_::const_iterator EQConstIter_;
	typedef std::vector<bool> UsedNodeIdMask_;
	//
	Status fetch_edges_(EdgeQueue_ &q, const Node_ &parent,
			    SizeType parent_depth) const
	{
		AMDA_ASSERT(q.empty());
		AMDA_ASSERT(parent.norm() > 0);

		Edge_ *last_edge = nullptr;
		const SizeType terminator = Traits_::get_terminator();
		SizeType char_of_edge = terminator;
		SizeType prev_char = terminator;

		for (SizeType i=parent.left(); i<parent.right(); i++) {
			const SizeType keylen = m_source->key_length(i);
			if (last_edge == nullptr && keylen == parent_depth) {
				AMDA_ASSERT(char_of_edge == terminator);
				AMDA_ASSERT(prev_char == terminator);
				AMDA_ASSERT(i==parent.left());
				// leaf.
			} else if (keylen <= parent_depth) {
				// not sorted nor unique.
				return S_INVALID_KEY;
			} else {
				// node.
				char_of_edge =
				    Traits_::char_to_node_offset(
					    m_source->key(i)[parent_depth]);
			}
			if (last_edge == nullptr ||
			    char_of_edge != prev_char) {
				// insert a new edge to the queue.
				if (last_edge)
					last_edge->node().right() = i;
				Edge_ newe(i, i, char_of_edge);
				q.push_back(newe);
				last_edge = &q.back();
				prev_char = char_of_edge;
			}
			AMDA_ASSERT(last_edge != nullptr);
		}
		AMDA_ASSERT(last_edge != nullptr);
		last_edge->node().right() = parent.right();
		return S_OK;
	}
	//
	SizeType fit_edges_(const EdgeQueue_ &q)
	{
		AMDA_ASSERT(!q.empty());

		const SizeType first_code = q.front().code();
		const SizeType last_code = q.back().code();

		SizeType pos = m_next_check_pos;
		bool first=true;
		if (pos < first_code) {
			first = false;
			pos = first_code + 1;
		}

		SizeType node;
		SizeType num_filled = 0;

		// find a fitting position which all elements corresponding to
		// the each edges of the node are free.
		for (;;) {
			if (m_array_factory->is_inuse(pos)) {
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
			m_array_factory->expand(node+last_code+1);
			// determine whether the all chars fit to the holes.
			for (EQConstIter_ i=q.begin()+1; i!=q.end(); ++i) {
				if (m_array_factory->is_inuse(node+i->code()))
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
		Status rv;

		AMDA_ASSERT(q.size() > 0);
		AMDA_ASSERT(m_used_node_id_mask.size() > node_id);
		AMDA_ASSERT(m_used_node_id_mask[node_id] == false);

		m_used_node_id_mask[node_id] = true;

		// ensure to reserve the elements before recursive call.
		for (EQConstIter_ i=q.begin(); i!=q.end(); ++i)
			m_array_factory->set_inuse(node_id, i->code());

		// insert descendants.
		for (EQConstIter_ i=q.begin(); i!=q.end(); ++i) {
			if (Traits_::is_terminator(i->code())) {
				// base[id + ch] expresses the edge to
				// the leaf node.
				AMDA_ASSERT(i->node().norm() == 1);
				AMDA_ASSERT(i==q.begin());
				m_array_factory->set_base(
					node_id,
					Traits_::get_terminator(),
					m_source->get_leaf_id(
						i->node().left()));
			} else {
				// base[id + ch]  expresses the edge to
				// the other node.
				SizeType child_node_id;
				rv = insert_children_(i->node(),
						      &child_node_id,
						      parent_depth+1);
				if (rv)
					return rv;
				m_array_factory->set_base(
					node_id, i->code(), child_node_id);
			}
		}
		return S_OK;
	}
	Status insert_children_(const Node_ &parent, SizeType *rnode_id,
				SizeType parent_depth)
	{
		Status rv;
		EdgeQueue_ q;
		SizeType node_id;

		rv = fetch_edges_(q, parent, parent_depth);
		if (rv)
			return rv;
		node_id = fit_edges_(q);
		rv = insert_edges_(node_id, q, parent_depth);
		if (rv)
			return rv;

		*rnode_id = node_id;

		return S_OK;
	}
public:
	~ScratchFactory() = default;
	ScratchFactory() = default;
	Status build(ArrayBody *rbody, const Source_ &src)
	{
		Status rv;
		SizeType node_id;

		if (src.num_entries() == 0)
			return S_NO_ENTRY;

		ArrayBodyFactory af;
		m_array_factory = &af;
		m_source = &src;
		m_next_check_pos = 1;
		UsedNodeIdMask_().swap(m_used_node_id_mask); // clear the mask

		m_used_node_id_mask.resize(1);
		m_used_node_id_mask[0] = true;

		m_array_factory->start();
		rv = insert_children_(RootNode_(*m_source), &node_id, 0);
		if (rv)
			return rv;

		// set base[0] to the root node ID, which should be 1.
		// note: node #0 is not a valid node.
		AMDA_ASSERT(node_id != 0);
		m_array_factory->set_base(0, Traits_::get_terminator(),
					  node_id);
		m_array_factory->done(rbody);
		m_array_factory = nullptr;
		m_source = nullptr;

		return S_OK;
	}
private:
	const Source_ *m_source = nullptr;
	ArrayBodyFactory *m_array_factory = nullptr;
	SizeType m_next_check_pos = 0;
	// already used node IDs.
	UsedNodeIdMask_ m_used_node_id_mask;
};


// ----------------------------------------------------------------------

template <class Traits_>
class PersistFactory
{
public:
	typedef typename Traits_::ArrayBody ArrayBody;
	typedef typename Traits_::SizeType SizeType;
	typedef typename Traits_::NodeIDType NodeIDType;
private:
	typedef typename Traits_::ArrayBody::PersistFactory ArrayBodyFactory;
public:
	template <class Source_>
	static Status build(ArrayBody *pbody, const Source_ &src)
	{
		ArrayBodyFactory af;
		return af.load(pbody, src);
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
class ArrayBody
{
	AMDA_NOCOPY(ArrayBody);
public:
	typedef typename Traits_::SizeType SizeType;
	typedef typename Traits_::NodeIDType NodeIDType;
	typedef typename Traits_::Storage Storage;
	class ScratchFactory;
	class PersistFactory;
public:
	//
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
	//
	~ArrayBody() = default;
	ArrayBody() = default;
	ArrayBody(Storage &s) : m_storage(s) { }
	void reset(typename Storage::HouseKeeper *hk =nullptr)
	{
		m_storage.reset(hk);
	}
	const Storage &storage() const { return m_storage; }
private:
	Storage m_storage;
};

template <class Traits_>
class ArrayBody<Traits_>::ScratchFactory
{
	AMDA_NOCOPY(ScratchFactory);
private:
	typedef typename Storage::ScratchFactory Storage_;
public:
	~ScratchFactory() = default;
	ScratchFactory() = default;
	void expand(SizeType sz) { m_storage.expand(sz); }
	bool is_inuse(SizeType idx) const
	{
		return
		    idx < m_storage.num_entries() &&
		    m_storage.is_inuse(idx);
	}
	void set_inuse(NodeIDType nid, SizeType ofs)
	{
		ofs += (SizeType)nid;
		AMDA_ASSERT(!m_storage.is_inuse(ofs));
		m_storage.expand(ofs+1);
		m_storage.set_inuse(ofs);
		m_storage.set_check(ofs, nid);
	}
	void set_base(NodeIDType base, SizeType ofs, NodeIDType nid)
	{
		ofs += (SizeType)base;
		this->expand(ofs+1);
		m_storage.set_base(ofs, nid);
	}
	void start() { m_storage.start(); }
	void done(ArrayBody *rbody)
	{ rbody->reset(m_storage.done()); }
private:
	Storage_ m_storage;
};


// ----------------------------------------------------------------------

template <class Traits_>
class ArrayBody<Traits_>::PersistFactory
{
public:
	template <class Source_>
	Status load(ArrayBody *rbody, const Source_ &src)
	{
		return src.load(rbody);
	}
};


// ----------------------------------------------------------------------

template <class Traits_>
class SortedKeySource
{
public:
	typedef typename Traits_::CharType CharType;
	typedef typename Traits_::SizeType SizeType;
	typedef typename Traits_::NodeIDType NodeIDType;
	typedef ScratchFactory<Traits_, SortedKeySource> Factory;
private:
	typedef const CharType *KeyType_;
public:
	~SortedKeySource() = default;
	SortedKeySource(SizeType ne,
			const KeyType_ *k,
			const SizeType *kl,
			const NodeIDType *lid = nullptr)
		: m_num_entries{ne}, m_keys{k}, m_key_lengths{kl},
		  m_leaf_ids{lid}
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
	typedef typename Traits_::ArrayBody ArrayBody_;
	typedef typename Traits_::Storage Storage_;
	typedef FileAccessorTmpl<Traits_, Storage_> Accessor_;
public:
	typedef PersistFactory<Traits_> Factory;
	//
	~FileSource() = default;
	FileSource(const std::string &fn) : m_filename(fn) { }
	Status load(ArrayBody_ *rbody) const
	{
		return Accessor_::load(m_filename, rbody);
	}
private:
	std::string m_filename;
};


template <class Traits_>
class FileDrain
{
private:
	typedef typename Traits_::ArrayBody ArrayBody_;
	typedef typename Traits_::Storage Storage_;
	typedef FileAccessorTmpl<Traits_, Storage_> Accessor_;
public:
	~FileDrain() = default;
	FileDrain(const std::string &fn) : m_filename(fn) { }
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
class SeparatedStorage
{
	AMDA_NOCOPY(SeparatedStorage);
public:
	typedef typename Traits_::SizeType SizeType;
	typedef typename Traits_::NodeIDType NodeIDType;
	typedef typename Traits_::BaseType BaseType;
	typedef typename Traits_::CheckType CheckType;
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
	explicit SeparatedStorage(HouseKeeper *hk)
		: m_house_keeper(hk), m_num_entries(hk->num_entries()),
		  m_bases(hk->bases()), m_checks(hk->checks())
	{
	}
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
class SeparatedStorage<Traits_>::ScratchFactory
{
	AMDA_NOCOPY(ScratchFactory);
private:
	typedef std::vector<BaseType> BaseArray_;
	typedef std::vector<CheckType> CheckArray_;
	class VariableSizedHouseKeeper_ : public HouseKeeper
	{
		friend class ScratchFactory;
	public:
		// HouseKeeper interface
		~VariableSizedHouseKeeper_() = default;
		SizeType num_entries() const { return m_num_entries; }
		const BaseType *bases() const { return &m_bases[0]; }
		const CheckType *checks() const { return &m_checks[0]; }
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
	HouseKeeper *done()
	{
		return m_house_keeper.release();
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
	typedef typename Traits_::SizeType SizeType;
	typedef typename Traits_::BaseType BaseType;
	typedef typename Traits_::CheckType CheckType;
private:
	typedef typename Traits_::ArrayBody ArrayBody_;
	typedef SeparatedStorage<Traits_> Storage_;
	typedef typename Storage_::HouseKeeper HouseKeeper_;
	class FixedSizedHouseKeeper_ : public HouseKeeper_
	{
	public:
		// for HouseKeeper interface
		~FixedSizedHouseKeeper_() = default;
		SizeType num_entries() const { return m_num_entries; }
		const BaseType *bases() const { return m_bases.get(); }
		const CheckType *checks() const { return m_checks.get(); }
		//
		FixedSizedHouseKeeper_(SizeType n)
			: m_num_entries(n),
			  m_bases(new BaseType [n]),
			  m_checks(new CheckType [n])
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
	static Status load(const std::string &fn, ArrayBody_ *rbody)
	{
		AMDA_ASSERT(rbody != nullptr);

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

		rbody->reset(hk.release());

		return S_OK;
	}
};


// ----------------------------------------------------------------------
// storage type which stores base and check into a structure of
// the array element.
//

template <class Traits_>
class StructuredStorage
{
	AMDA_NOCOPY(StructuredStorage);
public:
	typedef typename Traits_::SizeType SizeType;
	typedef typename Traits_::NodeIDType NodeIDType;
	typedef typename Traits_::BaseType BaseType;
	typedef typename Traits_::CheckType CheckType;
	typedef typename Traits_::ElementType ElementType;
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
	explicit StructuredStorage(HouseKeeper *hk)
		: m_house_keeper(hk), m_num_entries(hk->num_entries()),
		  m_elements(hk->elements())
	{
	}
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
class StructuredStorage<Traits_>::ScratchFactory
{
	AMDA_NOCOPY(ScratchFactory);
private:
	typedef std::vector<ElementType> ElementArray_;
	class VariableSizedHouseKeeper_ : public HouseKeeper
	{
		friend class ScratchFactory;
	public:
		// HouseKeeper interface
		~VariableSizedHouseKeeper_() = default;
		SizeType num_entries() const { return m_num_entries; }
		const ElementType *elements() const { return &m_elements[0]; }
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
	HouseKeeper *done()
	{
		return m_house_keeper.release();
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
	typedef typename Traits_::SizeType SizeType;
	typedef typename Traits_::ElementType ElementType;
private:
	typedef typename Traits_::ArrayBody ArrayBody_;
	typedef StructuredStorage<Traits_> Storage_;
	typedef typename Storage_::HouseKeeper HouseKeeper_;
	class FixedSizedHouseKeeper_ : public HouseKeeper_
	{
	public:
		// for HouseKeeper interface
		~FixedSizedHouseKeeper_() = default;
		SizeType num_entries() const { return m_num_entries; }
		const ElementType *elements() const
		{ return m_elements.get(); }
		//
		FixedSizedHouseKeeper_(SizeType n)
			: m_num_entries(n),
			  m_elements(new ElementType [n])
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
	static Status load(const std::string &fn, ArrayBody_ *rbody)
	{
		AMDA_ASSERT(rbody != nullptr);

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

		rbody->reset(hk.release());

		return S_OK;
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
	typedef CharType_ CharType;
	typedef SizeType_ SizeType;
	typedef NodeIDType_ NodeIDType;
	typedef ArrayBody<Traits> ArrayBody;
	typedef StorageType_<Traits> Storage;
	// the node offset for the terminator.
	static SizeType_ get_terminator() { return 0; }
	// whether the offset is terminator.
	static bool is_terminator(SizeType_ ofs) { return ofs == 0; }
	// convert character code to corresponding node offset.
	static SizeType_ char_to_node_offset(CharType ch)
	{ return (SizeType_)ch+1; }
	// whether the zone is too dense.
	static bool is_too_dense(SizeType num_filled, SizeType extent)
	{ return (float)num_filled / extent >= 0.95; }
	// for Storage
	typedef BaseType_ BaseType;
	typedef CheckType_ CheckType;
	typedef ElementType_ ElementType; // only for StructuredStorage
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
	typedef U8 CharType;
	typedef SizeType_ SizeType;
	typedef NodeIDType_ NodeIDType;
	typedef ArrayBody<Traits> ArrayBody;
	typedef SeparatedStorage<Traits> Storage;
	static SizeType_ get_terminator() { return 0; }
	static bool is_terminator(SizeType_ ofs) { return ofs == 0; }
	static SizeType_ char_to_node_offset(CharType ch)
	{ return (SizeType_)ch+1; }
	static bool is_too_dense(SizeType num_filled, SizeType extent)
	{ return (float)num_filled / extent >= 0.95; }
	typedef BaseType_ BaseType;
	typedef U8 CheckType;
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
