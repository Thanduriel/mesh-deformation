#pragma once

#include <utility>
#include <memory>

namespace util {

	template<typename T, int ElemPerBlock>
	class BlockAllocator
	{
	public:
		BlockAllocator()
			: first(new Node()), current(first.get())
		{

		}

		template<typename... Args>
		T* create(Args&&... args)
		{
			if (current->numElements == ElemPerBlock)
			{
				Node* prev = current;
				current = new Node();
				prev->next = current;
			}
			char* ptr = current->buffer + sizeof(T) * current->numElements;
			++current->numElements;

			return new(ptr)T(std::forward<Args>(args)...);
		}

		void reset()
		{
			first.reset(new Node());
			current = first.get();
		}

	private:
		struct Node
		{
			~Node()
			{
				T* ptr = reinterpret_cast<T*>(buffer);
				for (int i = 0; i < numElements; ++i)
					ptr[i].~T();

				if (next) delete next;
			}

			char buffer[sizeof(T) * ElemPerBlock];
			Node* next = nullptr;
			int numElements = 0;
		};

		std::unique_ptr<Node> first;
		Node* current;
	};
}