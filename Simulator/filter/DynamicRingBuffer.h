/**
* @file RingBuffer.h
*
* Declaration of class DynamicRingBuffer
*
* @author Oliver Urbann
*/

#ifndef __DynamicRingBuffer_h_
#define __DynamicRingBuffer_h_

#include <float.h>
/**
* @class DynamicRingBuffer
*
* template class for cyclic buffering of the last n values of Type V
*/
template <class V> class DynamicRingBuffer
{
public:
	/** Constructor */
	DynamicRingBuffer(const unsigned int _n) : buffer(0) {init(_n);}

	~DynamicRingBuffer()
	{
		if (buffer!=0)
			delete buffer;
	}

	/**
	* initializes the Ringbuffer
	*/
	void init (const unsigned int _n) {
		if (buffer==0)
			delete buffer;
		n=_n;
		buffer=new V[n];
		current = n - 1;
		numberOfEntries = 0;
		sum = 0;
	}

	/**
	* adds an entry to the buffer
	* \param v value to be added
	*/
	void add (const V v) 
	{
		if(numberOfEntries == n) sum -= getEntry(numberOfEntries - 1);
		sum += v;
		add();
		buffer[current] = v;
	}

	/**
	* adds an entry to the buffer.
	* The new head is not initialized, but can be changed afterwards.
	*/
	void add () 
	{
		current++;
		if (current==n) current=0;
		if (++numberOfEntries >= (int)n) numberOfEntries = n;
	}

	/**
	* removes the first added entry to the buffer
	*/
	void removeFirst () 
	{
		sum -= getEntry(numberOfEntries - 1);
		--numberOfEntries;
	}

	/**
	* returns an entry
	* \param i index of entry counting from last added (last=0,...)
	* \return a reference to the buffer entry
	*/
	V getEntry (int i)
	{
		int j = current - i;
		j %= (int)n;
		if (j < 0) j += n;
		return buffer[j];
	}

	/**
	* returns an entry
	* \param v the value the entry i shall be updated with
	* \param i index of entry counting from last added (last=0,...)
	*/
	void updateEntry(const V v, int i)
	{
		int j = current - i;
		j %= (int)n;
		if (j < 0) j += n;
		buffer[j] = v;
	}

	/**
	* returns an entry
	* \param i index of entry counting from last added (last=0,...)
	* \return a reference to the buffer entry
	*/
	V operator[] (int i)
	{
		return getEntry(i);
	}

	/**
	* returns a constant entry.
	* \param i index of entry counting from last added (last=0,...)
	* \return a reference to the buffer entry
	*/
	const V operator[] (int i) const
	{
		return buffer[i > current ? n + current - i : current - i];
	}

	/** Returns the number of elements that are currently in the ring buffer
	* \return The number
	*/
	int getNumberOfEntries() const
	{
		return numberOfEntries;
	}

	V getSum()
    {
      return sum;
    }

	/**
	 * returns the average value of all entries
	 * \return the average value
	 */
	V getAverage()
	{
      // Return 0 if buffer is empty
      if (0==numberOfEntries) return 0;

	  return (sum / numberOfEntries);
	}



protected:
	int current;
	int numberOfEntries;
	V *buffer;
	unsigned int n;

	V sum;
};
#endif // __RingBuffer_h_
