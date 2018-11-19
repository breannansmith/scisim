#ifndef VALIDATING_SPIN_BOX
#define VALIDATING_SPIN_BOX

#include <QSpinBox>

#include "scisim/Math/Rational.h"

class ValidatingSpinBox final : public QSpinBox
{

  Q_OBJECT

public:

  ValidatingSpinBox( QWidget* parent );

  virtual ~ValidatingSpinBox() override = default;

  ValidatingSpinBox( ValidatingSpinBox& ) = delete;
  ValidatingSpinBox( ValidatingSpinBox&& ) = delete;
  ValidatingSpinBox& operator=( const ValidatingSpinBox& ) = delete;
  ValidatingSpinBox& operator=( ValidatingSpinBox&& ) = delete;

  void setTimestep( const Rational<std::intmax_t>& dt );

  virtual QValidator::State	validate( QString& text, int& pos ) const override;

  virtual void fixup( QString& input ) const override;

public slots:

  void displayErrorMessage();

private:

  QWidget* m_parent;
  Rational<std::intmax_t> m_dt;
  // NB: This is a temporary hack to export error message from validate so we can use them in displayErrorMessage
  mutable QString error_message;

};

#endif
