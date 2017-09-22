#include "Mode.h"

Mode::Mode()
{
}

Mode::~Mode()
{
	for (int i = 0; i < _classifierList.size(); i++)
	{
		delete _classifierList[i];
	}
}