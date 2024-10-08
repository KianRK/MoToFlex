
/**
 * \file RCObject.h
 *
 * \brief Base class for Reference Counted Objects
 * 
 * \authors Marc Nunkesser (taken from More Effective C++ by Scott Meyers)
 */

/* $log$ */

#ifndef RCOBJECT_H
#define RCOBJECT_H

/**
 * \brief Base class for Reference Counted Objects.
 * 
 * This base class contains functionality that enables the derived classes to
 * be handled by Refernce Counted Pointers (as described in <i>Scott Meyers</i>,
 * <i>More Effective C++</i>). The explicit use of this methods should not be
 * necessary, they should only be used by RCPtr objects.
 */

class RCObject {
public:
	
	/**
	 * \brief Adds a reference.
	 * 
	 * i.e. the reference counter is increased by one.
	 */
	void addReference();

	/**
	 * \brief Removes a reference to this object.
	 * 
	 * The reference counter is decreased by one. If the counter reaches zero,
	 * the object will be deleted. 
	 */
	void removeReference();
	
	/*
	 * \brief Marks object as unshareable.
	 * 
	 * \todo has unsharable any effect at all?!?
	 */
	//void markUnshareable();

	/*
	 * \brief Indicates if object is marked as shareable.
	 */
	//bool isShareable() const;
	
	/**
	 * \brief Indicates if object shared between two or more Refernce Counted
	 * Pointers (RCPtr).
	 * 
	 * \return true iff more than one reference exists.
	 */
	bool isShared() const;

protected:
	
	/// default constructor
	RCObject();
	
	/**
	 * \brief copy constructor
	 * 
	 * The new object will be marked as shareable and the reference counter set
	 * to zero.
	 * 
	 * @param rhs reference to the object to be copied
	 */
	RCObject(const RCObject& rhs);
	
	/// assignment operator
	RCObject& operator=(const RCObject& rhs);
	
	/// destructor
	virtual ~RCObject()=0;

private:
	int refCount;
	bool shareable;
};


#endif
