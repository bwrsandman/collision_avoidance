#include <array>
#include <chrono>
#include <cstdio>
#include <memory>
#include <string_view>

#include <RVOSimulator.h>
#include <SDL.h>
#include <SDL_render.h>
#include <imgui.h>
#include <imgui_impl_sdl.h>
#include <imgui_sdl.h>

#if __EMSCRIPTEN__
#include <emscripten.h>
#endif

struct Simulation
{
  enum configuration_t
  {
    CIRCLE,
    DEADLOCK,

    _CONFIGURATION_COUNT,
  };
  static constexpr std::array<std::string_view, _CONFIGURATION_COUNT>
    configuration_strings{
      "Circle",
      "Deadlock",
    };
  struct options_t
  {
    configuration_t configuration{ CIRCLE };
    bool run_simulation{ false };
    bool show_goal{ false };
    float time_scale{ 10.0f };
    float neighborDist{ 15.0f };
    int maxNeighbors{ 10 };
    float timeHorizon{ 10.0f };
    float timeHorizonObst{ 10.0f };
    float radius{ 1.5f };
    float maxSpeed{ 10.0f };
    int numAgents{ 250 };
    float circleRadius{ 200 };
  };
  Simulation() = default;

  void initialize(const options_t& options)
  {
    simulator = std::make_unique<RVO::RVOSimulator>();
    /* Specify the default parameters for agents that are subsequently added. */
    simulator->setAgentDefaults(options.neighborDist,
                                options.maxNeighbors,
                                options.timeHorizon,
                                options.timeHorizonObst,
                                options.radius,
                                options.maxSpeed);

    for (const auto& obstacle : obstacles) {
      simulator->addObstacle(obstacle);
    }
    if (!obstacles.empty()) {
      simulator->processObstacles();
    }

    /*
     * Add agents, specifying their start position, and store their goals on the
     * opposite side of the environment.
     */
    goals.clear();
    if (options.configuration == CIRCLE) {
      goals.reserve(options.numAgents);
      for (size_t i = 0; i < options.numAgents; ++i) {
        simulator->addAgent(
          options.circleRadius *
          RVO::Vector2(std::cos(i * 2 * M_PI / options.numAgents),
                       std::sin(i * 2 * M_PI / options.numAgents)));
        goals.push_back(-simulator->getAgentPosition(i));
      }
    } else if (options.configuration == DEADLOCK) {
      simulator->addAgent(RVO::Vector2(-2 * options.radius, 0));
      goals.emplace_back(10 * options.radius, 0);
      simulator->addAgent(RVO::Vector2(2 * options.radius, 0));
      goals.emplace_back(-10 * options.radius, 0);
    }
  }

  void set_preferred_velocities()
  {
    for (int i = 0; i < static_cast<int>(simulator->getNumAgents()); ++i) {
      RVO::Vector2 goalVector = goals[i] - simulator->getAgentPosition(i);

      simulator->setAgentPrefVelocity(i, goalVector);
    }
  }

  void step(float dt)
  {
    /* Specify the global time step of the simulation. */
    simulator->setTimeStep(dt);
    simulator->doStep();
  }

  void commit_obstacle()
  {
    if (staging_obstacle.size() > 2) {
      simulator->addObstacle(staging_obstacle);
      simulator->processObstacles();
      obstacles.emplace_back(staging_obstacle);
    }
    staging_obstacle.clear();
  }

  std::unique_ptr<RVO::RVOSimulator> simulator;
  std::vector<RVO::Vector2> goals;
  std::vector<RVO::Vector2> staging_obstacle;
  std::vector<std::vector<RVO::Vector2>> obstacles;
};

struct Renderer
{
  struct options_t
  {
    float scale{ 1.5f };
    float offset_x{ 300 };
    float offset_y{ 0 };
  };

public:
  Renderer()
    : window(nullptr)
    , renderer(nullptr)
    , ui(nullptr)
  {}
  ~Renderer() { terminate(); }

  bool initialize()
  {
    if (SDL_Init(SDL_INIT_VIDEO) != 0) {
      std::printf("Failed to initialize SDL\n");
      return false;
    }

    window = SDL_CreateWindow(
      "Collision Avoidance", SDL_HINT_DEFAULT, SDL_HINT_DEFAULT, 1280, 768, 0);
    if (!window) {
      std::printf("Failed to create SDL window\n");
      terminate();
      return false;
    }

    renderer = SDL_CreateRenderer(window, -1, SDL_RENDERER_ACCELERATED);
    if (!renderer) {
      std::printf("Failed to create SDL renderer\n");
      terminate();
      return false;
    }

    int w, h;
    SDL_GetWindowSize(window, &w, &h);
    width = w;
    height = h;

    // Setup Dear ImGui context
    IMGUI_CHECKVERSION();
    ui = ImGui::CreateContext();
    if (!ui) {
      std::printf("Failed to create ImGui context\n");
      terminate();
      return false;
    }

    // Setup Platform/Renderer bindings
    ImGuiSDL::Initialize(renderer, width, height);
    ImGui_ImplSDL2_InitForOpenGL(window, nullptr);

    return true;
  }

  void terminate() noexcept
  {
    if (ui) {
      ImGuiSDL::Deinitialize();
      ImGui_ImplSDL2_Shutdown();
      ImGui::DestroyContext(ui);
      ui = nullptr;
    }

    if (renderer) {
      SDL_DestroyRenderer(renderer);
      renderer = nullptr;
    }
    if (window) {
      SDL_DestroyWindow(window);
      window = nullptr;
    }
    SDL_Quit();
  }

  RVO::Vector2 toScreenSpace(const RVO::Vector2& point)
  {
    return {
      width / 2 + options.offset_x + point.x() * options.scale,
      height / 2 + options.offset_y - point.y() * options.scale,
    };
  }

  RVO::Vector2 fromScreenSpace(const RVO::Vector2& point)
  {
    return RVO::Vector2{
      point.x() - width / 2 - options.offset_x,
      -point.y() + height / 2 + options.offset_y,
    } / options.scale;
  }

  void draw(float dt,
            Simulation& simulation,
            Simulation::options_t& simulation_options)
  {
    ImGui_ImplSDL2_NewFrame(window);
    ImGui::NewFrame();

    ImGui::Begin("Controls");
    ImGui::Text("dt: %.5f seconds", dt);
    ImGui::Text("Keyboard controls:\n"
                "\tSpacebar: Pause/Continue Simulation.\n"
                "\tBackspace: Reset Simulation.\n"
                "Mouse controls:\n"
                "\tDouble click to add obstacle vertex\n"
                "\tRight click to finish obstacle");

    ImGui::SliderFloat("Zoom", &options.scale, 0.01, 100, "%.3f", 2.0f);
    float offset_max =
      static_cast<float>(std::max(width, height)) * 0.5f +
      (simulation_options.circleRadius + simulation_options.radius) *
        options.scale;
    ImGui::SliderFloat2(
      "Offset", &options.offset_x, -offset_max, offset_max + 1);
    ImGui::SliderFloat(
      "Time Scale", &simulation_options.time_scale, 0.01, 100, "%.3f", 2.0f);
    ImGui::SliderFloat(
      "Neighbor Distance (m)", &simulation_options.neighborDist, 0, 50);
    ImGui::SliderInt("Max Neighbors", &simulation_options.maxNeighbors, 0, 50);
    ImGui::SliderFloat(
      "Tau for other agents (s)", &simulation_options.timeHorizon, 0, 50);
    ImGui::SliderFloat(
      "Tau for Obstacles (s)", &simulation_options.timeHorizonObst, 0, 50);
    ImGui::SliderFloat("Agent Radius (m)", &simulation_options.radius, 0, 10);
    ImGui::SliderFloat(
      "Agent Max Speed (m/s)", &simulation_options.maxSpeed, 0, 100);
    ImGui::SliderInt("Number of Agents", &simulation_options.numAgents, 0, 500);
    ImGui::SliderFloat(
      "Radius of Circle (m)", &simulation_options.circleRadius, 0, 1000);

    ImGui::Checkbox("Show Goal", &simulation_options.show_goal);
    ImGui::Checkbox("Run Simulation", &simulation_options.run_simulation);

    auto item_current =
      Simulation::configuration_strings[simulation_options.configuration];
    if (ImGui::BeginCombo("Configuration", item_current.data())) {
      for (int n = 0; n < Simulation::configuration_strings.size(); n++) {
        bool is_selected =
          (item_current == Simulation::configuration_strings[n]);
        if (ImGui::Selectable(Simulation::configuration_strings[n].data(),
                              is_selected)) {
          simulation_options.configuration =
            static_cast<Simulation::configuration_t>(n);
        }
        if (is_selected)
          ImGui::SetItemDefaultFocus();
      }
      ImGui::EndCombo();
    }

    if (!simulation.staging_obstacle.empty()) {
      if (ImGui::Button("Add Obstacle")) {
        simulation.commit_obstacle();
      }
    }

    if (ImGui::Button("Reset")) {
      simulation.initialize(simulation_options);
    }
    ImGui::End();

    const SDL_Rect clip = {
      0, 0, static_cast<int>(width), static_cast<int>(height)
    };
    SDL_RenderSetClipRect(renderer, &clip);

    SDL_SetRenderDrawColor(renderer, 0, 0, 0, SDL_ALPHA_OPAQUE);
    SDL_RenderClear(renderer);

    if (simulation_options.show_goal) {
      SDL_SetRenderDrawColor(renderer, 0x3F, 0x3F, 0x3F, SDL_ALPHA_OPAQUE);
      for (uint32_t i = 0; i < simulation.simulator->getNumAgents(); ++i) {
        auto point = toScreenSpace(simulation.simulator->getAgentPosition(i));
        auto goal = toScreenSpace(simulation.goals[i]);
        SDL_RenderDrawLine(renderer, point.x(), point.y(), goal.x(), goal.y());
      }
    }

    SDL_SetRenderDrawColor(renderer, 0xFF, 0xFF, 0xFF, SDL_ALPHA_OPAQUE);
    for (uint32_t i = 0; i < simulation.simulator->getNumAgents(); ++i) {
      auto point = toScreenSpace(simulation.simulator->getAgentPosition(i));
      int w = static_cast<int>(simulation_options.radius * 2 * options.scale);
      int h = static_cast<int>(simulation_options.radius * 2 * options.scale);
      if (w > 1 && h > 1) {
        SDL_Rect rect{
          static_cast<int>(point.x() -
                           simulation_options.radius * options.scale),
          static_cast<int>(point.y() -
                           simulation_options.radius * options.scale),
          w,
          h,
        };
        SDL_RenderDrawRect(renderer, &rect);
      } else {
        SDL_RenderDrawPoint(renderer, point.x(), point.y());
      }
    }

    // Obstacles
    SDL_SetRenderDrawColor(renderer, 0x7F, 0x7F, 0x7F, SDL_ALPHA_OPAQUE);
    RVO::Vector2 previous;
    for (const auto& obstacle : simulation.obstacles) {
      if (obstacle.size() < 3) {
        continue;
      }
      previous = toScreenSpace(obstacle.back());
      for (auto i : obstacle) {
        auto point = toScreenSpace(i);
        SDL_RenderDrawLine(
          renderer, previous.x(), previous.y(), point.x(), point.y());
        previous = point;
      }
    }

    // Staging obstacle
    SDL_SetRenderDrawColor(renderer, 0x7F, 0x7F, 0x7F, SDL_ALPHA_OPAQUE);
    for (uint32_t i = 0; i < simulation.staging_obstacle.size(); ++i) {
      auto point = toScreenSpace(simulation.staging_obstacle[i]);
      SDL_RenderDrawPoint(renderer, point.x(), point.y());
      if (i > 0) {
        SDL_RenderDrawLine(
          renderer, previous.x(), previous.y(), point.x(), point.y());
      }
      previous = point;
    }
    if (ui_want_capture_mouse()) {
      if (simulation.staging_obstacle.size() > 2) {
        auto point_1 = toScreenSpace(simulation.staging_obstacle[0]);
        auto point_2 = toScreenSpace(simulation.staging_obstacle.back());
        SDL_RenderDrawLine(
          renderer, point_1.x(), point_1.y(), point_2.x(), point_2.y());
      }
    } else {
      if (!simulation.staging_obstacle.empty()) {
        int x, y;
        SDL_GetMouseState(&x, &y);
        auto point = toScreenSpace(simulation.staging_obstacle[0]);
        SDL_RenderDrawLine(renderer, point.x(), point.y(), x, y);
      }
      if (simulation.staging_obstacle.size() > 1) {
        int x, y;
        SDL_GetMouseState(&x, &y);
        auto point = toScreenSpace(simulation.staging_obstacle.back());
        SDL_RenderDrawLine(renderer, point.x(), point.y(), x, y);
      }
    }

    ImGui::Render();
    ImGuiSDL::Render(ImGui::GetDrawData());

    SDL_RenderPresent(renderer);
  }

  void resolution(uint32_t& out_width, uint32_t& out_height) const
  {
    out_width = width;
    out_height = height;
  }

  bool ui_want_capture_mouse() const
  {
    ImGuiIO& io = ImGui::GetIO();
    return io.WantCaptureMouse;
  }

  bool ui_want_capture_keyboard() const
  {
    ImGuiIO& io = ImGui::GetIO();
    return io.WantCaptureKeyboard;
  }

  SDL_Window* window;
  SDL_Renderer* renderer;
  ImGuiContext* ui;
  options_t options;
  uint32_t width;
  uint32_t height;
};

#if __EMSCRIPTEN__
void
em_main_loop_callback(void* arg);
#endif

class App
{
public:
  App()
    : time_stamp(std::chrono::high_resolution_clock::now())
  {
    simulation.initialize(simulation_options);
  }
  virtual ~App() = default;

  bool main_loop()
  {
    auto now = std::chrono::high_resolution_clock::now();
    auto dt = std::chrono::duration_cast<std::chrono::duration<float>>(
      now - time_stamp);
    time_stamp = now;

    renderer.draw(dt.count(), simulation, simulation_options);

    if (simulation_options.run_simulation) {
      simulation.set_preferred_velocities();
      simulation.step(simulation_options.time_scale * dt.count());
    }

    SDL_Event event;
    while (SDL_PollEvent(&event)) {
      ImGui_ImplSDL2_ProcessEvent(&event);
      uint32_t width, height;
      renderer.resolution(width, height);
      if (event.type == SDL_QUIT) {
        return false;
      } else if (event.type == SDL_MOUSEBUTTONDOWN &&
                 !renderer.ui_want_capture_mouse()) {
        if (event.button.button == SDL_BUTTON_LEFT &&
            event.button.clicks == 2) {
          simulation.staging_obstacle.emplace_back(renderer.fromScreenSpace(
            RVO::Vector2(event.button.x, event.button.y)));
        } else if (event.button.button == SDL_BUTTON_RIGHT) {
          simulation.staging_obstacle.emplace_back(renderer.fromScreenSpace(
            RVO::Vector2(event.button.x, event.button.y)));
          simulation.commit_obstacle();
        }
      } else if (event.type == SDL_MOUSEWHEEL &&
                 !renderer.ui_want_capture_mouse()) {
        renderer.options.scale += event.wheel.y * 0.01 * renderer.options.scale;
        if (renderer.options.scale < 0) {
          renderer.options.scale = 0.0f;
        }
      } else if (event.type == SDL_MOUSEMOTION &&
                 event.motion.state == SDL_PRESSED &&
                 !renderer.ui_want_capture_mouse()) {
        renderer.options.offset_x += event.motion.xrel;
        renderer.options.offset_y += event.motion.yrel;
      } else if (event.type == SDL_KEYDOWN &&
                 !renderer.ui_want_capture_keyboard()) {
        switch (event.key.keysym.scancode) {
          case SDL_SCANCODE_ESCAPE:
            return false;
          case SDL_SCANCODE_SPACE:
            simulation_options.run_simulation =
              !simulation_options.run_simulation;
            break;
          case SDL_SCANCODE_BACKSPACE:
            simulation.initialize(simulation_options);
            break;
        }
      }
    }
    return true;
  }

  int run()
  {
    if (!renderer.initialize()) {
      return EXIT_FAILURE;
    }

    uint32_t width, height;
    renderer.resolution(width, height);

#if __EMSCRIPTEN__
    emscripten_set_main_loop_arg(em_main_loop_callback, this, 0, true);
#else
    while (main_loop()) {
    }
#endif

    terminate();

    return EXIT_SUCCESS;
  }

  void terminate() { renderer.terminate(); }

private:
  std::chrono::time_point<
#if __EMSCRIPTEN__
    std::chrono::steady_clock
#else
    std::chrono::system_clock
#endif
    >
    time_stamp;
  Renderer renderer;
  Simulation::options_t simulation_options;
  Simulation simulation;
};

#if __EMSCRIPTEN__
void
em_main_loop_callback(void* arg)
{
  auto app = reinterpret_cast<App*>(arg);
  if (!app->main_loop()) {
    app->terminate();
    emscripten_cancel_main_loop();
  }
}
#endif

int
main(int argc, char* argv[])
{
  App app;
  return app.run();
}
