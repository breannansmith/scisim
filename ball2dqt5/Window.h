#ifndef WINDOW_H
#define WINDOW_H

#include <QMainWindow>

class QKeyEvent;
class ContentWidget;

class Window final : public QMainWindow
{

public:

  Window( const QString& scene_name = "", QWidget* parent = nullptr );
  virtual ~Window() override = default;
  Window( Window& ) = delete;
  Window( Window&& ) = delete;
  Window& operator=( const Window& ) = delete;
  Window& operator=( Window&& ) = delete;

};

#endif
