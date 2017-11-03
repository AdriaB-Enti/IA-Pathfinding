#pragma once
#include "Connection.h"

Connection::Connection()
{
}

Connection::~Connection()
{
}

float Connection::GetCost()
{
	return 0.0f;
}

Vector2D Connection::getFromNode()
{
	return Vector2D();
}

Vector2D Connection::getToNode()
{
	return Vector2D();
}
