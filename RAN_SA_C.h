#ifndef RAN_SA_C_H
#define RAN_SA_C_H

#include "Segmentation.h"
#pragma once

class RAN_SA_C : public Segmentation // RAN_SA_C is a Segmentation
{
public:
	/// Default Constructor
	RAN_SA_C(const string &);
	
	/// Set functions
	/// model parameter set function
	void setModelParameter(string);

	/// Get functions
	/// model parameter get function
	string getModelParameter(void);
	
	/// RANSAC algorithm function
	void ransacAlgorithm(void);

	/// Default Destructor
	~RAN_SA_C();


private:
	/// Data members
	/// model parameter data member
	string modelParameter;
	
};

#endif