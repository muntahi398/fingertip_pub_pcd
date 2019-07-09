#ifndef Buffer_h
#define Buffer_h

class Buffer
{
	private: int capacity;
	private: unsigned char* data;
	private: int length;
	
	public: int GetCapacity();
	private: void SetCapacity(int value);
	public: unsigned char* GetData();
	public: int GetLength();
	
	public: Buffer();
	public: Buffer(unsigned char*, int);
	public: Buffer(const char*);
	public: ~Buffer();
	
	public: void Append(Buffer*);
	public: void Clear();
	public: int Find(Buffer*);
	public: Buffer* Extract(Buffer*, Buffer*);
	public: Buffer* SubBuffer(int startIndex, int endIndex);
	public: void CopyTo(void*);

	private: void Initialize();
	private: void Initialize(unsigned char*, int);

	public: static void Copy(unsigned char*, unsigned char*, int, int = 0);
	public: static bool Equals(unsigned char*, unsigned char*, int);
};

#endif
