#ifndef WINDOW_H
#define WINDOW_H

#include <QMainWindow>

struct RenderSettings;
struct SimSettings;

class Window final : public QMainWindow
{

public:

  Window( const QString& scene_name, SimSettings& sim_settings, RenderSettings& render_settings, QWidget* parent = nullptr );
  virtual ~Window() override = default;
  Window( Window& ) = delete;
  Window( Window&& ) = delete;
  Window& operator=( const Window& ) = delete;
  Window& operator=( Window&& ) = delete;

};

#endif
