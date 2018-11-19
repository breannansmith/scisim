#include "ValidatingSpinBox.h"

#include <QMessageBox>

#include "scisim/Math/MathDefines.h"

ValidatingSpinBox::ValidatingSpinBox( QWidget* parent )
: QSpinBox( parent )
, m_parent( parent )
, m_dt( 0, 1 )
, error_message()
{}

void ValidatingSpinBox::setTimestep( const Rational<std::intmax_t>& dt )
{
  m_dt = dt;
}

QValidator::State	ValidatingSpinBox::validate( QString& text, int& pos ) const
{
  QValidator::State parent_state = QSpinBox::validate( text, pos );
  if( QSpinBox::validate( text, pos) != QValidator::Acceptable )
  {
    return parent_state;
  }

  QString fixed = text;
  QSpinBox::fixup(fixed);
  const int fps = fixed.toInt();

  // Flag invalid entries as intermediate
  if( 1.0 < scalar( m_dt * std::intmax_t( fps ) ) )
  {
    return QValidator::Intermediate;
  }
  else
  {
    const Rational<std::intmax_t> potential_steps_per_frame{ std::intmax_t( 1 ) / ( m_dt * std::intmax_t( fps ) ) };
    if( !potential_steps_per_frame.isInteger() )
    {
      if( m_dt != Rational<std::intmax_t>{ 0 } )
      {
        return QValidator::Intermediate;
      }
    }
  }

  return QValidator::Acceptable;
}

void ValidatingSpinBox::fixup( QString& input ) const
{
  QSpinBox::fixup(input);

  const int fps = input.toInt();

  if( 1.0 < scalar( m_dt * std::intmax_t( fps ) ) )
  {
    error_message = tr("Requested FPS faster than timestep. Retaining old value.");
  }
  else
  {
    const Rational<std::intmax_t> potential_steps_per_frame{ std::intmax_t( 1 ) / ( m_dt * std::intmax_t( fps ) ) };
    if( !potential_steps_per_frame.isInteger() )
    {
      if( m_dt != Rational<std::intmax_t>{ 0 } )
      {
        error_message = tr("Timestep and FPS do not yield an integer number of timesteps. Retaining old value.");
      }
    }
  }
}

void ValidatingSpinBox::displayErrorMessage()
{
  // Block here is a workaround for: https://bugreports.qt.io/browse/QTBUG-40
  this->blockSignals(true);
  if( !error_message.isEmpty() )
  {
    QMessageBox::warning( m_parent, tr("SCISim 2D Ball Simulation"), error_message );
    error_message.clear();
  }
  this->blockSignals(false);
}
