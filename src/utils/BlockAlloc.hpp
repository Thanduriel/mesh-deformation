#pragma once

#include <utility>
#include <memory>

namespace util {

	/* A simple allocator for objects of a single type.
	 * @param T The type of objects to create.
	 * @param ElemPerBlock How many elements should fit into a single block.
	 *		  A larger value leads to more wasted space, but fewer allocations.
	 */
	template<typename T, int ElemPerBlock>
	class BlockAllocator
	{
	public:
		BlockAllocator()
			: first(new Node()), current(first.get())
		{

		}

		// Construct an object of type T in place.
		// Maintains ownership, to delete the object use reset().
		template<typename... Args>
		T* create(Args&&... args)
		{
			// create new block
			if (current->numElements == ElemPerBlock)
			{
				Node* prev = current;
				current = new Node();
				prev->next = current;
			}
			char* ptr = current->buffer + sizeof(T) * current->numElements;
			++current->numElements;

			return new (ptr) T(std::forward<Args>(args)...);
		}

		// Deletes all owned objects and frees all but one block.
		void reset()
		{
			first->reset();
			current = first.get();
		}

	private:
		struct Node
		{
			void reset()
			{
				// destroy elements
				T* ptr = reinterpret_cast<T*>(buffer);
				for (int i = 0; i < numElements; ++i)
					ptr[i].~T();

				if (next) delete next;
			}
			~Node()
			{
				reset();
			}

			char buffer[sizeof(T) * ElemPerBlock];
			Node* next = nullptr;
			int numElements = 0;
		};

		std::unique_ptr<Node> first;
		Node* current;
	};
}