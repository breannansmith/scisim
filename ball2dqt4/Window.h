#ifndef WINDOW_H
#define WINDOW_H

#include <QMainWindow>

class QKeyEvent;
class ContentWidget;

class Window final : public QMainWindow
{

public:

  Window( QWidget* parent = nullptr );

  virtual void keyPressEvent( QKeyEvent* event ) override;

protected:

  virtual void closeEvent( QCloseEvent* event ) override;

private:

  ContentWidget* m_content_widget;

};

#endif
