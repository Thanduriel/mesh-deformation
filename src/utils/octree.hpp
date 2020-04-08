#pragma once

#include "blockalloc.hpp"
#include <pmp/MatVec.h>
#include <memory>

namespace util {

	// A sparse octree for 3D points.
	template<typename T, int BucketSize, typename FloatT = pmp::Scalar>
	class Octree
	{
	public:
		using Key = pmp::Vector<FloatT, 3>;

		Octree() : rootNode_(allocator_.create(Key(0,0,0),1.0) ) {}

		void insert(const Key& key, const T& el)
		{
			Key center = rootNode_->center_;
			FloatT size = rootNode_->size_;
			while (!is_in(key, center, size))
			{
				int index = child_index(center, key);
				center -= CENTER_SHIFTS[index] * size;
				size *= 2;

				TreeNode* newRoot = allocator_.create(center, size);
				newRoot->childs_[index] = rootNode_;
				rootNode_ = newRoot;
			}

			rootNode_->insert(key, el, allocator_);
		}
		
		// remove all elements
		void clear()
		{
			allocator_.reset();
			rootNode_ = allocator_.create(Key(0, 0, 0), 1.0);
		}

		/* layout of Processor
			struct TreeProcessor
			{
				bool descend(const Key& center, FloatT size);
				void process(const Key& key, const T& el);
			};
		*/
		template<class Processor>
		void traverse(Processor& proc)
		{
			rootNode_->traverse(proc);
		}
	
	private:
		
		struct TreeNode
		{
			TreeNode(const Key& center, FloatT size) 
				: center_{center}, size_(size), childs_{}, numElements_(0)
			{}

			void insert(const Key& key, const T& el, BlockAllocator<TreeNode, 128>& alloc_)
			{
				if (numElements_ < BucketSize)
					data_[numElements_++] = std::pair{key, el};
				else
				{
					const int index = child_index(key, center_);
					if (!childs_[index])
					{
						const FloatT size = size_ / 2.0;
						childs_[index] = alloc_.create(center_ + CENTER_SHIFTS[index] * size, size);
					}
					childs_[index]->insert(key, el, alloc_);
				}
			}

			template<class Processor>
			void traverse(Processor& proc)
			{
				if (!proc.descend(center_,size_)) return;

				for (int i = 0; i < numElements_; ++i)
					proc.process(data_[i].first, data_[i].second);

				for (int i = 0; i < 8; ++i)
					if (childs_[i]) childs_[i]->traverse(proc);
			}

			Key center_;
			FloatT size_;
			TreeNode* childs_[8];
			int numElements_;
			std::pair<Key,T> data_[BucketSize];
		};

		static bool is_in(const Key& el, const Key& center, FloatT size)
		{
			return center[0] - size < el[0] && center[0] + size >= el[0]
				&& center[1] - size < el[1] && center[1] + size >= el[1]
				&& center[2] - size < el[2] && center[2] + size >= el[2];
		}

		static int child_index(const Key& el, const Key& center)
		{
			return (center[0] < el[0] ? 1 : 0) + (center[1] < el[1] ? 2 : 0) + (center[2] < el[2] ? 4 : 0);
		}

		static inline const Key CENTER_SHIFTS[8] = {
			Key(-1,-1,-1), Key(1,-1,-1), Key(-1,1,-1), Key(1,1,-1),
			Key(-1,-1,1), Key(1,-1,1), Key(-1,1,1), Key(1,1,1) };

		BlockAllocator<TreeNode, 128> allocator_;
		TreeNode* rootNode_;
	};
}