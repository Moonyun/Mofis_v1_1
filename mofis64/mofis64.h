#pragma once

#include "resource.h"

//¹ıÂË
enum Enum_property {
	Enum_diameter,
	Enum_area,
	Enum_perimeter,
	Enum_volume,
	Enum_eccentricity,
	Enum_roundness
};

struct Condition
{
	Enum_property m_property;
	float m_min;
	float m_max;
	int m_b_and;
};