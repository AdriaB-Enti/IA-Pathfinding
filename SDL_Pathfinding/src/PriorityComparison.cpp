#include "PriorityComparison.h"



PriorityComparison::PriorityComparison(const bool& revparam=false)
{
	reverse = revparam;
}


bool PriorityComparison:: operator()(const int& lhs, const int&rhs)const {
	if (reverse) return (lhs > rhs);
	else return (lhs<rhs);
}