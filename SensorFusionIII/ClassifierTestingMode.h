#ifndef CLASSIFIER_TESTING_MODE_H
#define CLASSIFIER_TESTING_MODE_H

#include "Mode.h"

class ClassifierTestingMode : public Mode
{
private:

public:
	ClassifierTestingMode();
	virtual ~ClassifierTestingMode();
	void Run();
};

#endif