#pragma once
class PriorityComparison
{
	bool reverse;
public:
	PriorityComparison(const bool&);
	bool operator()(const int&,const int&)const;
};

