//===========================================================================
/*!
 *  \file Svm.h
 *
 *  \brief Support Vector Machine interface
 *
 *  \author  T. Glasmachers
 *  \date    2005
 *
 *  \par Copyright (c) 1999-2007:
 *      Institut f&uuml;r Neuroinformatik<BR>
 *      Ruhr-Universit&auml;t Bochum<BR>
 *      D-44780 Bochum, Germany<BR>
 *      Phone: +49-234-32-25558<BR>
 *      Fax:   +49-234-32-14209<BR>
 *      eMail: Shark-admin@neuroinformatik.ruhr-uni-bochum.de<BR>
 *      www:   http://www.neuroinformatik.ruhr-uni-bochum.de<BR>
 *
 *
 *
 *  <BR><HR>
 *  This file is part of Shark. This library is free software;
 *  you can redistribute it and/or modify it under the terms of the
 *  GNU General Public License as published by the Free Software
 *  Foundation; either version 2, or (at your option) any later version.
 *
 *  This library is distributed in the hope that it will be useful,
 *  but WITHOUT ANY WARRANTY; without even the implied warranty of
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
 *  GNU General Public License for more details.
 *
 *  You should have received a copy of the GNU General Public License
 *  along with this library; if not, write to the Free Software
 *  Foundation, Inc., 675 Mass Ave, Cambridge, MA 02139, USA.
 */
//===========================================================================


#ifndef _Svm_H_
#define _Svm_H_


#include <SharkDefs.h>
#include <ReClaM/Model.h>
#include <ReClaM/Optimizer.h>
#include <ReClaM/KernelFunction.h>
#include <ReClaM/QuadraticProgram.h>
#include <Rng/GlobalRng.h>

class SvmApproximation;


//!
//! \brief Support Vector Machine (SVM) as a ReClaM #Model
//!
//! \author T. Glasmachers
//! \date 2005
//!
//! \par The SVM class provides a Support Vector Machine as a
//! parametric #Model, that is, it computes a linear expansion
//! of the kernel function with fixed training examples as one
//! component. It can be viewed as a parametrized family of maps
//! from the input space to the reals.
//!
//! \par The parameter array of the SVM class defines the
//! affine linear solution in feature space, usually described
//! as a vector \f$ \alpha \f$ and a real valued offset b.
//! Note that different SVM training procedures impose
//! constraints on the possible values these parameters are
//! allowed to take.
//!
//! \par In ReClaM, the SVM as a model is used for prediction
//! only. That is, it does not impose any training scheme and
//! could in theory be training using any error measure and any
//! optimizer. In practice, one wants to apply standard SVM
//! training schemes to efficiently find the SVM solution.
//! Therefore, the SVM should be trained with the special
//! optimizer derived class #C_SVM. This class implements the
//! so-called C-SVM as a training scheme and uses a quadratic
//! program solver to obtain the optimal solution.
//! However, some other training schemes like basic
//! regularization networks and gaussian process work on the
//! same type of model. Therefore the SVM should be understood
//! as an affine linear model in the kernel induced feature
//! space, rather than being too closely connected to standard
//! SVM training schemes.
//!
class SVM : public Model
{
public:
	//! Constructor
	//!
	//! \param  pKernel      kernel function to use for training and prediction
	//! \param  bSignOutput  true if the SVM should output binary labels, false if it should output real valued function evaluations
	SVM(KernelFunction* pKernel, bool bSignOutput = false);

	//! Constructor
	//!
	//! \param  pKernel      kernel function to use for training and prediction
	//! \param  input        training data points
	//! \param  bSignOutput  true if the SVM should output binary labels, false if it should output real valued function evaluations
	SVM(KernelFunction* pKernel, const Array<double>& input, bool bSignOutput = false);

	//! Destructor
	~SVM();

	//! \par
	//! As the SVM can be constructed without training data
	//! points, although these are needed for the computation
	//! of the model, this member makes the training data
	//! known to the SVM. This method is usually called by
	//! the #SVM_Optimizer class, but in case a model was
	//! loaded from a file it can be necessary to invoke the
	//! function manually.
	//!
	//! \par
	//! A side effects, the method eventually rescales the
	//! parameter vector. In any case, all parameters are reset.
	//! The number of examples and the input space dimension
	//! are overwritten.
	//!
	//! \author  T. Glasmachers
	//! \date    2006
	//!
	//! \param  input        training data points
	//! \param  copy         maintain a copy of the input data
	void SetTrainingData(const Array<double>& input, bool copy = false);

	//! compute the SVM prediction on data
	//! \author  T. Glasmachers
	//! \date    2006
	//!
	void model(const Array<double>& input, Array<double>& output);

	//! compute the SVM prediction on data
	//! \author  T. Glasmachers
	//! \date    2006
	//!
	double model(const Array<double>& input);

	//! \par
	//! The modelDerivative member computes the derivative
	//! of the SVM function w.r.t. its parameters. Although
	//! this information is probably never used, it is easy
	//! to compute :)
	//!
	//! Of course, the derivative does not make sense after
	//! the application of the sign function. Thus, this method
	//! always returns the derivative w.r.t. the real valued
	//! SVM output.
	//!
	//! \author  T. Glasmachers
	//! \date    2006
	//!
	void modelDerivative(const Array<double>& input, Array<double>& derivative);

	//! \par
	//! The modelDerivative member computes the derivative
	//! of the SVM function w.r.t. its parameters. Although
	//! this information is probably never used, it is easy
	//! to compute :)
	//!
	//! Of course, the derivative does not make sense after
	//! the application of the sign function. Thus, this method
	//! always returns the derivative w.r.t. the real valued
	//! SVM output.
	//!
	//! \author  T. Glasmachers
	//! \date    2006
	//!
	void modelDerivative(const Array<double>& input, Array<double>& output, Array<double>& derivative);

	//! \par
	//! retrieve one of the coeffitients from the solution
	//! vector usually referred to as \f$ \alpha \f$
	//!
	//! \author  T. Glasmachers
	//! \date    2006
	//!
	inline double getAlpha(int index)
	{
		return parameter(index);
	}

	//! \par
	//! retrieve the solution offset, usually referred to
	//! as \f$ b \f$
	//!
	//! \author  T. Glasmachers
	//! \date    2006
	//!
	inline double getOffset()
	{
		return parameter(examples);
	}

	//! return the kernel function object
	inline KernelFunction* getKernel()
	{
		return kernel;
	}

	//! return the training data points
	inline const Array<double>& getPoints()
	{
		return *x;
	}

	//! return the number of training examples
	inline unsigned int getExamples()
	{
		return examples;
	}

	//! return the input space dimension
	inline unsigned int getDimension()
	{
		return inputDimension;
	}

	//!
	//! \brief Discard zero coefficients and corresponding training data
	//!
	//! \par
	//! If the SVM object onws its training data array,
	//! it can make itself sparse after training.
	//!
	void MakeSparse();

	//! \par
	//! Load the complete SVM model including
	//! kernel parameters, alpha, b and the
	//! support vectors.
	//!
	//! \author  T. Glasmachers
	//! \date    2006
	//!
	bool LoadSVMModel(std::istream& is);

	//! \par
	//! Save the complete SVM model including
	//! kernel parameters, alpha, b and the
	//! support vectors.
	//!
	//! \author  T. Glasmachers
	//! \date    2006
	//!
	bool SaveSVMModel(std::ostream& os);

	//! \par
	//! Import a libsvm 2.81 model file.
	//!
	//! \author  T. Glasmachers
	//! \date    2006
	//!
	//! \return  On success a SVM model is returned.
	//!          A return value of NULL indicates an error.
	//!
	static SVM* ImportLibsvmModel(std::istream& is);

	//! \par
	//! Import an SVM-light model file.
	//!
	//! \author  T. Glasmachers
	//! \date    2006
	//!
	//! \return  On success a SVM model is returned.
	//!          A return value of NULL indicates an error.
	//!
	static SVM* ImportSvmlightModel(std::istream& is);

	friend class SvmApproximation;

protected:
	//! \brief Read a token from a stream
	//!
	//! \par
	//! The function reads a token from a stream.
	//! It stores the token into a null terminated string.
	//! If the first character read is already a separator
	//! the function returns an empty token.
	//!
	//! \param  is          stream to read from
	//! \param  buffer      buffer to store the token
	//! \param  maxlength   size of the buffer
	//! \param  separators  characters terminating the token
	//!
	//! \return A status code is returned:
	//! <ul>
	//!   <li> token read successfully --> terminating char  </li>
	//!   <li> token read successfully, end of file --> 1001 </li>
	//!   <li> unrecoverable error --> 1002 </li>
	//!   <li> buffer overrun, token truncated to size maxlength-1 --> 1003 </li>
	//! </ul>
	static int ReadToken(std::istream& is, char* buffer, int maxlength, const char* separators);

	//! \brief Discard characters with stopping condition
	//!
	//! \par
	//! The function reads characters from a stream until
	//! one of the terminating characters is found.
	//!
	//! \param  is          stream to read from
	//! \param  separators  termination characters
	//!
	//! \return A status code is returned:
	//! <ul>
	//!   <li> terminating char  </li>
	//!   <li> end of file --> 1001 </li>
	//!   <li> unrecoverable error --> 1002 </li>
	//! </ul>
	static int DiscardUntil(std::istream& is, const char* separators);

	//! kernel function
	KernelFunction* kernel;

	//! true if x and y are allocated by LoadModel
	bool bOwnMemory;

	//! training data points
	const Array<double>* x;

	//! true if the SVM outputs the binary label \f$ \pm 1 \f$ only,
	//! false if the SVM outputs the value of the linear feature space function.
	bool signOutput;

	//! number of training data points #x and labels #y
	unsigned int examples;
};


//!
//! \brief Multi Class Support Vector Machine Model
//!
//! \author T. Glasmachers
//! \date 2007
//!
//! \par The MultiClassSVM class provides a Support Vector
//! Machine as a parametric Model, that is, it computes a
//! linear expansion of the kernel function with fixed
//! training examples as one component. It can be viewed
//! as a parametrized family of maps from the input space
//! to the output space. In this formulation the output
//! space is some \f$ R^n \f$.
//!
//! \par In constrast to the binary classification or
//! regression SVM the MultiClassSVM maps inputs not to
//! the reals but to a vector space. For each class there
//! exists a prototype label vector in this space. The
//! classifier predicts the class label whose prototype
//! vector has the largest inner product to the output
//! vector. That is, the label prototype most similar to
//! the output vector is used for prediction.
//!
//! \par Due to the fact that the number of classes is
//! always finite we use the linear kernel on the outputs
//! without loss of generality. However, there are two
//! concurrent standard initializations of the label
//! prototypes: Either the standard basis of \f$ R^n \f$
//! can be used, or the corners of a symmetric simplex in
//! \f$ R^{n-1} \f$ centered at the origin, which
//! coincides with the labels \f$ \pm 1 \in R \f$ of the
//! binary case.
//!
//! \par The parameters of this machine are organized
//! into one decision function per class. Therefore there
//! exist as many parameters as there are training
//! examples times classes. For some formulations of
//! multi class SVMs some of these are dummy variables
//! which are always zero. Furthermore, for most training
//! procedures available by now the offset or bias term
//! is set to zero.
//!
class MultiClassSVM : public Model
{
public:
	//! Constructor
	//! \param  pKernel             input space kernel
	//! \param  numberOfClasses     number of classes with indices starting from 0
	//! \param  bOrthogonalVectors  true: standard basis prototypes; false: simplex prototypes
	//! \param  bNumberOutput       true: output class index; false: output vector
	MultiClassSVM(KernelFunction* pKernel, unsigned int numberOfClasses, bool bOrthogonalVectors, bool bNumberOutput = true);

	//! Constructor
	//! \param  pKernel        input space kernel
	//! \param  prototypes     label prototype vectors
	//! \param  bNumberOutput  true: output class index; false: output vector
	MultiClassSVM(KernelFunction* pKernel, Array<double> prototypes, bool bNumberOutput = true);

	//! Destructor
	~MultiClassSVM();


	//! \brief Make the training data known to the SVM.
	//!
	//! The training data are needed in order to
	//! represent the SVM model because the
	//! parameter vector stores only coefficients
	//! relative to these data.
	//!
	//! \param  input        training data points
	//! \param  copy         maintain a copy of the input data
	void SetTrainingData(const Array<double>& input, bool copy = false);

	//! compute the SVM prediction on data
	void model(const Array<double>& input, Array<double>& output);

	//! compute the SVM prediction on data
	unsigned int model(const Array<double>& input);

	//! normalize the class-wise decision functions
	void Normalize();

	//! return the kernel function object
	inline KernelFunction* getKernel()
	{
		return kernel;
	}

	//! return the training data points
	inline const Array<double>& getPoints()
	{
		return *x;
	}

	//! return the coefficient of a given example and label
	//! \param  index  index of the corresponding training example
	//! \param  c      zero-based class index (0, ..., classes-1)
	inline double getAlpha(unsigned int index, unsigned int c) const
	{
		return parameter(classes * index + c);
	}

	//! return the solution offset vector
	//! \param  c  zero-based class index (0, ..., classes-1)
	inline double getOffset(unsigned int c) const
	{
		return parameter(classes * examples + c);
	}

	//! return the number of classes
	inline unsigned int getClasses() const
	{
		return classes;
	}

	//! return the class representing prototype vector
	//! \param  c  zero-based class index (0, ..., classes-1)
	inline const ArrayReference<double> getClassPrototype(unsigned int c) const
	{
		return prototypes[c];
	}

	//! convert a label vector into a class index
	unsigned int VectorToClass(const Array<double>& v);

protected:
	void Predict(const Array<double>& input, Array<double>& output);
	void Predict(const Array<double>& input, ArrayReference<double> output);

	//! kernel function
	KernelFunction* kernel;

	//! true if x and y are allocated by LoadModel
	bool bOwnMemory;

	//! training data points
	const Array<double>* x;

	//! true if the SVM outputs the class label only
	//! false if the SVM outputs the value of the linear feature space function.
	bool numberOutput;

	//! number of training data points and labels
	unsigned int examples;

	//! number of classes
	unsigned int classes;

	//! vectors associated with the classes
	Array<double> prototypes;
};


//!
//! \brief Base class of all meta models for SVM training
//!
//! \par
//! The MetaSVM is the base class of all SVM training schemes.
//! It stores the hyperparameters as its model parameters.
//! These usually include the kernel parameters and at least
//! one complexity control parameter.
//!
//! \author T. Glasmachers
//! \date 2007
//!
class MetaSVM : public Model
{
public:
	//! Constructor
	//!
	//! \param  pSVM                     Pointer to the SVM to be optimized.
	//! \param  numberOfHyperParameters  number of hyperparameters additional to the kernel parameters
	MetaSVM(SVM* pSVM, unsigned int numberOfHyperParameters);

	//! Constructor
	//!
	//! \param  pSVM                     Pointer to the MultiClassSVM to be optimized.
	//! \param  numberOfHyperParameters  number of hyperparameters additional to the kernel parameters
	MetaSVM(MultiClassSVM* pSVM, unsigned int numberOfHyperParameters);

	//! Descructor
	~MetaSVM();


	//! return the underlying SVM model
	inline SVM* getSVM()
	{
		return dynamic_cast<SVM*>(svm);
	}

	//! return the underlying MultiClassSVM model
	inline MultiClassSVM* getMultiClassSVM()
	{
		return dynamic_cast<MultiClassSVM*>(svm);
	}

	//! Just calls the underlying SVM
	void model(const Array<double>& input, Array<double>& output);

	//! overloaded version of Model::setParameter
	void setParameter(unsigned int index, double value);

	//! ensure the kernel is feasible
	bool isFeasible();

protected:
	//! pointer to the underlying #SVM model
	Model* svm;

	//! pointer to the kernel function object
	KernelFunction* kernel;

	//! number of hyperparameters of the SVM training scheme
	unsigned int hyperparameters;
};


//!
//! \brief Meta Model for SVM training
//!
//! \author T. Glasmachers
//! \date 2006
//!
//! \par
//! The C-SVM is a training scheme for Support Vector Machines.
//! It defines a positive constant C controlling the solution
//! complexity.
//!
//! \par
//! In the 1-norm SVM formulation, this constant
//! bounds the \f$ alpha_i \f$ parameters of the SVM from above,
//! limiting their contribution to the solution.
//! For the 2-norm SVM formulation, the \f$ alpha_i \f$ are not
//! bounded from above. Instead, the kernel matrix is reglarized
//! by adding \f$ 1 / C \f$ to the diagonal entries.
//!
//! \par
//! The C_SVM class is a meta model which is based on an #SVM
//! model. Its parameter vector is composed of the SVM hyper-
//! parameters, i.e., the complexity parameter C and the
//! parameters of a #KernelFunction object. As a meta model it
//! does not make sense to call the #model or #modelDerivative
//! members of this class. However, the #modelDerivative function
//! has been implemented to compute the derivative of the
//! underlying SVM model w.r.t. the hyperparameters stored in
//! the C_SVM model. See #PrepareDerivative and #modelDerivative
//! for further reference.
//!
//! \par
//! For unbalanced data, it has proven fertile to consider class
//! specific complexity parameters \f$ C_+ \f$ and \f$ C_- \f$
//! instead of a single constant C. These parameters are usually
//! coupled by the equation \f$ \ell_- C_+ = \ell_+ C_- \f$, where
//! \f$\ell_{\pm}\f$ are the class magnitudes. Therefore, the C_SVM
//! model introduces only the \f$ C_+ \f$ parameter as a model
//! parameter, whereas \f$ C_- = \ell_- / \ell_+ C_+ \f$ can be
//! computed from this parameter. As only the \f$ C_+ \f$ parameter
//! is subject to optimization, all derivatives have to be computed
//! w.r.t \f$ C_+ \f$, even if \f$ C_- \f$ is used for the
//! computation.
//!
//! \par
//! The parameter \f$ C_+ \f$ can be stored in a way allowing for
//! unconstrained optimization, that is, the exponential function
//! is used to compute the value from the model parameter. As a
//! side effect, this parameterization is more natural and
//! therefore probably better suited for optimization.
//!
class C_SVM : public MetaSVM
{
public:
	//! Constructor
	//!
	//! \param  pSVM     Pointer to the SVM to be optimized.
	//! \param  Cplus    initial value of \f$ C_+ \f$
	//! \param  Cminus   initial value of \f$ C_- \f$
	//! \param  norm2    true if 2-norm slack penalty is to be used
	//! \param  unconst  true if the parameters are to be represented as \f$ \log(C) \f$. This allows for unconstrained optimization.
	C_SVM(SVM* pSVM, double Cplus, double Cminus, bool norm2 = false, bool unconst = false);

	//! Descructor
	~C_SVM();


	//! Prepare the computation of modelDerivative.
	//! This method computes and stores the derivatives
	//! of the SVM parameters w.r.t. the regularization
	//! and kernel parameters for later reference by
	//! the modelDerivative function.
	void PrepareDerivative();

	//! Computes the derivative of the model w.r.t.
	//! regularization and kernel parameters.
	//! Be sure to call PrepareDerivative after SVM
	//! training and before calling this function!
	void modelDerivative(const Array<double>& input, Array<double>& derivative);

	//! overloaded version of Model::setParameter
	void setParameter(unsigned int index, double value);

	//! return the parameter C for positive class examples
	inline double get_Cplus()
	{
		return C_plus;
	}

	//! return the parameter C for negative class examples
	inline double get_Cminus()
	{
		return C_minus;
	}

	//! return true if the 2-norm slack penalty is in use
	inline bool is2norm()
	{
		return norm2penalty;
	}

	//! return true if the 2-norm slack penalty is in use
	inline bool isUnconstrained()
	{
		return exponential;
	}

	//! return the quotient \f$ C_- / C_+ \f$
	inline double getCRatio()
	{
		return C_ratio;
	}

	//! ensure C is positive
	bool isFeasible();

protected:
	//! true if the 2-norm slack penalty is to be used
	bool norm2penalty;

	//! Complexity constant used for positive training examples
	double C_plus;

	//! Complexity constant used for negative training examples
	double C_minus;

	//! fraction \f$ C_- / C_+ \f$
	double C_ratio;

	//! if true the C-parameters are computed via \f$ C = exp(\tilde C) \f$ .
	bool exponential;

	Array<double> alpha_b_Derivative;
};


//!
//! \brief Meta Model for SVM training
//!
//! \author T. Glasmachers
//! \date 2007
//!
//! \par
//! The \f$ \varepsilon \f$-SVM is a training scheme for Support
//! Vector Machines for regression. It defines a positive constant
//! \f$ \varepsilon \f$ controlling loss accuracy and another
//! positive constant C controlling the solution complexity.
//!
//! \par
//! The Epsilon_SVM class is a meta model which is based on an
//! #SVM model. Its parameter vector is composed of the SVM hyper-
//! parameters, that is the hyperparameters C and \f$ \varepsilon \f$,
//! and the parameters of a #KernelFunction object. As a meta
//! model it does not make sense to call the #model or
//! #modelDerivative members of this class.
//!
//! \par
//! The parameters C and \f$ \varepsilon \f$ can be stored in a
//! way allowing for unconstrained optimization, that is, the
//! exponential function is used to compute the value from the
//! model parameter. As a side effect, this parameterization is
//! more natural and therefore probably better suited for
//! optimization.
//!
class Epsilon_SVM : public MetaSVM
{
public:
	//! Constructor
	//!
	//! \param  pSVM     Pointer to the SVM to be optimized.
	//! \param  Cplus    initial value of \f$ C_+ \f$
	//! \param  Cminus   initial value of \f$ C_- \f$
	//! \param  epsilon  initial value of \f$ \varepsilon \f$
	//! \param  unconst  true if the parameters are to be represented as \f$ \log(C) \f$ and \f$ \log(\varepsilon) \f$. This allows for unconstrained optimization.
	Epsilon_SVM(SVM* pSVM, double C, double epsilon, bool unconst = false);

	//! Descructor
	~Epsilon_SVM();


	//! overloaded version of Model::setParameter
	void setParameter(unsigned int index, double value);

	//! return the complexity control parameter C
	inline double get_C()
	{
		return C;
	}

	//! return the regression loss parameter \f$ \varepsilon \f$
	inline double get_epsilon()
	{
		return epsilon;
	}

	//! ensure C and epsilon are positive
	bool isFeasible();

protected:
	//! complexity control parameter C
	double C;

	//! regression loss parameter \f$ \varepsilon \f$
	double epsilon;

	//! if set the parameters are computed via exp.
	bool exponential;
};



//!
//! \brief Meta Model for SVM training
//!
//! \author E. Diner
//! \date 2008
//!
//! \par
//! The one class SVM is a training scheme for classification and density estimation.
//! It defines a positive constant \f$ \nu \f$ allows to misclassify a certain fraction
//! of the training pattern which lie closest to the origin.
//! For more information read: Estimating the Support of a High-Dimensional Distribution, B. Schölkopf
class OneClassSVM : public MetaSVM
{
public:

	//! \param pSVM     Pointer to the SVM to be optimized.
	//! \param  nu		fraction of the training data that can be misclassified
	OneClassSVM(SVM* pSVM, double fractionNu);

	//! Descructor
	~OneClassSVM();

	//! overloaded version of Model::setParameter
	void setParameter(unsigned int index, double value);

	//! return regularization parameter \f$ \nu \f$
	inline double getNu()
	{
		return nu;
	}

	//! ensure nu is positive
	bool isFeasible();

protected:
	//! Regualrization parameter, which directly controls the fraction of misclassified training pattern (outlier).
	double nu;
};


//!
//! \brief Meta Model for SVM training
//!
//! \author T. Glasmachers
//! \date 2007
//!
//! \par
//! The Regularization Network is a special training scheme for
//! SVM training. It simply minimizes the regularized mean
//! squared error resulting in a quadratic program without any
//! constraints.
//!
class RegularizationNetwork : public MetaSVM
{
public:
	//! Constructor
	RegularizationNetwork(SVM* pSVM, double gamma);

	//! Destructor
	~RegularizationNetwork();


	inline double get_gamma() { return parameter(0); }
	inline void set_gamma(double gamma) { setParameter(0, gamma); }

	//! ensure gamma is positive
	bool isFeasible();
};


//!
//! \brief Meta Model for SVM training
//!
//! \author T. Glasmachers
//! \date 2008
//!
//! \par
//! Canonical multi class SVM meta model, covering the machines
//! proposed independently by Weston and Watkins and by Wahba.
//!
class AllInOneMcSVM : public MetaSVM
{
public:
	//! Constructor
	AllInOneMcSVM(MultiClassSVM* pSVM, double C);

	//! Destructor
	~AllInOneMcSVM();


	inline double get_C() { return parameter(0); }
	inline void set_C(double C) { setParameter(0, C); }

	//! ensure C is positive
	bool isFeasible();
};


//!
//! \brief Meta Model for SVM training
//!
//! \author T. Glasmachers
//! \date 2008
//!
//! \par
//! Meta model for the multi class SVM proposed by
//! Crammer and Singer.
//!
class CrammerSingerMcSVM : public MetaSVM
{
public:
	//! Constructor
	//!
	//! \par
	//! The parameter \f$ \beta \f$ (roughly)
	//! corresponds to 2/C in other machines.
	CrammerSingerMcSVM(MultiClassSVM* pSVM, double beta);

	//! Destructor
	~CrammerSingerMcSVM();


	inline double get_beta() { return parameter(0); }
	inline void set_beta(double beta) { setParameter(0, beta); }

	//! ensure beta is positive
	bool isFeasible();
};


//!
//! \brief Meta Model for SVM training
//!
//! \author T. Glasmachers
//! \date 2008
//!
//! \par
//! Set of binary machines trained
//! with the all-versus-one rule.
//!
class OVAMcSVM : public MetaSVM
{
public:
	//! Constructor
	OVAMcSVM(MultiClassSVM* pSVM, double C);

	//! Destructor
	~OVAMcSVM();


	inline double get_C() { return parameter(0); }
	inline void set_C(double C) { setParameter(0, C); }

	//! ensure C is positive
	bool isFeasible();
};


//!
//! \brief Meta Model for SVM training
//!
//! \author T. Glasmachers
//! \date 2008
//!
//! \par
//! Multi-Class SVM training at the cost of
//! a binary classification machine.
//!
//! S. Szedmak, S. Shawe-Taylor, E. Parado-Hernandez,
//! "Learning via Linear Operators: Maximum Margin Regression; Multiclass and Multiview Learning at One-class Complexity",
//! Technical report, PASCAL, Southampton, UK (2006)
//!
class OCCMcSVM : public MetaSVM
{
public:
	//! Constructor
	OCCMcSVM(MultiClassSVM* pSVM, double C);

	//! Destructor
	~OCCMcSVM();


	inline double get_C() { return parameter(0); }
	inline void set_C(double C) { setParameter(0, C); }

	//! ensure C is positive
	bool isFeasible();
};


//!
//! \brief Optimizer for SVM training by quadratic programming
//!
//! \author T. Glasmachers
//! \date 2006
//!
//! Although the SVM_Optimizer fits into the ReClaM concept
//! of an #Optimizer, it should not be used the usual way,
//! for three reasons:
//! <ul>
//!   <li>It does not depend on the error function object
//!     provided as a parameter to #optimize.</li>
//!   <li>It does not return a useful error value.
//!     The quadratic program optimum is returned,
//!     which is not suitable for model selection.</li>
//!   <li>It is not an iterative optimizer. The first call to
//!     #optimize already finds the optimal solution.</li>
//! </ul>
//! Nevertheless, the SVM_Optimizer is one of the most important
//! classes in the SVM context, as it calls the #QuadraticProgram
//! class to solve the SVM dual problem, computing the #SVM
//! coefficients \f$ alpha \f$ and b.
//!
class SVM_Optimizer : public Optimizer
{
public:
	//! Constructor
	SVM_Optimizer();

	//! Destructor
	~SVM_Optimizer();


	//! \brief Default initialization
	//!
	//! \par
	//! This is the default #Model initialization method.
	//! If the model parameter is not a valid #MetaSVM
	//! model, the method will throw an exception.
	//!
	//! \param  model    Meta model containing complexity constant and kernel
	void init(Model& model);

	//! \brief Default #Optimizer interface
	//!
	//! \par
	//! The optimize member uses the QuadraticProgram class to train the
	//! Support Vector Machine. If the model parameter supplied
	//! is not a valid #SVM object, the method checks for a
	//! #C_SVM object to determine the unserlying #SVM.
	//! Otherwise it throws an exception.
	//!
	//! \par
	//! Note that no #ErrorFunction object is needed for SVM
	//! training. If no ErrorFunction object is available,
	//! the static dummy reference #dummyError can be used.
	//!
	//! \param   model   The #SVM model to optimize
	//! \param   error   The error object is not used at all. The #dummyError can be used.
	//! \param   input   Training input used for the optimization
	//! \param   target  Training labels used for the optimization
	//! \return  As there is no error evaluation, the function returns
	//!          the dual target function value.
	double optimize(Model& model, ErrorFunction& error, const Array<double>& input, const Array<double>& target);

	//! \brief Trains the #SVM with the given dataset.
	//!
	//! \par
	//! This version of the optimize member is well suited for
	//! #SVM training, but it does not fit into the ReClaM concept.
	//! Thus, a less specialized version of optimize is available,
	//! which overrides the corresponding #Optimizer member.
	//!
	//! \param  model   The #SVM model to optimize
	//! \param  input   Training input used for the optimization
	//! \param  target  Training labels used for the optimization
	//! \param  copy    Maintain a copy of the input data in the SVM object
	//! \return  As there is no error evaluation, the function returns
	//!          the dual target function value.
	double optimize(SVM& model, const Array<double>& input, const Array<double>& target, bool copy = false);

	//! \brief Trains the #MultiClassSVM with the given dataset.
	//!
	//! \par
	//! This version of the optimize member is well suited for
	//! #MultiClassSVM training, but it does not fit into the
	//! ReClaM concept. Thus, a less specialized version of optimize
	//! is available, which overrides the corresponding #Optimizer
	//! member.
	//!
	//! \param  model   The #MultiClassSVM model to optimize
	//! \param  input   Training input used for the optimization
	//! \param  target  Training labels used for the optimization
	//! \param  copy    Maintain a copy of the input data in the MultiClassSVM object
	//! \return  As there is no error evaluation, the function returns zero.
	void optimize(MultiClassSVM& model, const Array<double>& input, const Array<double>& target, bool copy = false);

	//! The static member dummyError is provided for calls to the
	//! #optimize method in case there is no #ErrorFunction object
	//! available. Do not use or call any members of dummyError,
	//! as it is an invalid reference. It is only provided for
	//! making syntactically correct calls to the #optimize method.
	static ErrorFunction& dummyError;

	//! Return the #QpSvmDecomp used by the previous call to #optimize.
	//! The #QuadraticProgram object may be of use for the calling
	//! procedure as it proviedes some solution statistics as the
	//! number of iterations used for training and the final objective
	//! function value.
	inline QPSolver* get_Solver()
	{
		return solver;
	}

	//! Set the solution accuracy
	inline void setAccuracy(double accuracy = 0.001)
	{
		this->accuracy = accuracy;
	}

	//! Set the maximum number of iterations
	//! for the quadratic program solver
	inline void setMaxIterations(SharkInt64 maxiter = -1)
	{
		this->maxIter = maxiter;
	}

	//! Set the maximum number of seconds
	//! for the quadratic program solver
	inline void setMaxSeconds(int seconds = -1)
	{
		maxSeconds = seconds;
	}

	//! Set the verbosity
	inline void setVerbose(bool verbose = true)
	{
		printInfo = verbose;
	}

	//! Set the cache size in megabytes
	inline void setCacheSize(unsigned int cacheSize)
	{
		cacheMB = cacheSize;
	}

	inline bool isOptimal()
	{
		return optimal;
	}

protected:
	enum eMode
	{
		eC1,				// C-SVM with 1-norm penalty for binary classification
		eC2,				// C-SVM with 2-norm penalty for binary classification
		eEpsilon,			// \varepsilon-SVM for regression
		eNu,				// \nu-SVM, not implemented yet
		eRegularizationNetwork,
		eGaussianProcess,	// Gaussian Process
		e1Class,			// One-Class SVM for density estimation XXX
		eAllInOne,			// standard multi class SVM
		eCrammerSinger,		// MC-SVM by Crammer and Singer
		eOVA,				// one-versus-all multi class SVM
		eOCC,		// binary-SVM-cost multi-class SVM
	};

	//! mode of operation
	eMode mode;

	//! kernel matrix
	QPMatrix* matrix;

	//! kernel matrix with cache
	CachedMatrix* cache;

	//! quadratic program solver used in the last #optimize call
	QPSolver* solver;

	//! upper bound for the variables corresponding to positive class examples
	double Cplus;

	//! upper bound for the variables corresponding to negative class examples
	double Cminus;

	//! upper bound C and lower bound -C for regression coefficients
	double C;

	//! parameter of the \f$ \varepsilon \f$-insensitive regression loss function
	double epsilon;

	//! gamma parameter of the regularization network
	double gamma;

	//! beta parameter of the crammer and singler SVM
	double beta;

	//! fraction of outliers for One-Class SVM XXX
	double fractionOfOutliers;

	//! upper bound for all One Class SVM Lagrange multipliers XXX
	double OneClassBoxUpper;

	//! should the quadratic program solver output its progress?
	bool printInfo;

	//! kernel cache size in megabytes used by the quadratic program solver
	unsigned int cacheMB;

	//! accuracy up to which the KKT conditions have to be fulfilled
	double accuracy;

	//! maximum number of #QpSvmDecomp iterations
	SharkInt64 maxIter;

	//! maximum time for the #QpSvmDecomp
	int maxSeconds;

	//! variable that checks if the solver converged
	bool optimal;
};


typedef SVM KernelExpansion;
typedef SVM LinearKernelModel;


#endif

