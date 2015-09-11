// PythonScripting.cpp
//
// Breannan Smith
// Last updated: 09/10/2015

#include "PythonScripting.h"

PythonScripting::PythonScripting()
{}

void PythonScripting::restitutionCoefficient( const std::vector<std::unique_ptr<Constraint>>& active_set, VectorXs& cor )
{}

void PythonScripting::frictionCoefficient( const std::vector<std::unique_ptr<Constraint>>& active_set, VectorXs& mu )
{}

void PythonScripting::startOfSim()
{}

void PythonScripting::endOfSim()
{}

void PythonScripting::startOfStep( const unsigned next_iteration, const Rational<std::intmax_t>& dt )
{}

void PythonScripting::endOfStep( const unsigned next_iteration, const Rational<std::intmax_t>& dt )
{}

std::string PythonScripting::name() const
{
  return "";
}
