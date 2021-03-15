////////////////////////////////////////////////////////////////////////////////
/**
@file       FlockingDriver.cpp
@brief      CS 561

            Main application file for doing agent-based flocking simulation

@note       Instructor: Jason Hanson
@note       Copyright © 2021 DigiPen Institute of Technology
*/
////////////////////////////////////////////////////////////////////////////////
#include <iostream>
#include <thread>
#include <chrono>
#include <SDL2/SDL.h>
#include <GL/glew.h>
#include <GL/gl.h>
#include <glm/glm.hpp>
#include <glm/gtc/matrix_transform.hpp>
#include "Flocker.h"
#include "Geometry.h"
#include "arcball_camera.h"

#include "imgui/imgui.h"
#include "imgui/imgui_impl_sdl.h"
#include "imgui/imgui_impl_opengl3.h"

#if defined(IMGUI_IMPL_OPENGL_LOADER_GL3W)
#include <GL/gl3w.h>            // Initialize with gl3wInit()
#elif defined(IMGUI_IMPL_OPENGL_LOADER_GLEW)
#include <GL/glew.h>            // Initialize with glewInit()
#elif defined(IMGUI_IMPL_OPENGL_LOADER_GLAD)
#include <glad/glad.h>          // Initialize with gladLoadGL()
#elif defined(IMGUI_IMPL_OPENGL_LOADER_GLBINDING2)
#define GLFW_INCLUDE_NONE       // GLFW including OpenGL headers causes ambiguity or multiple definition errors.
#include <glbinding/Binding.h>  // Initialize with glbinding::Binding::initialize()
#include <glbinding/gl/gl.h>
using namespace gl;
#elif defined(IMGUI_IMPL_OPENGL_LOADER_GLBINDING3)
#define GLFW_INCLUDE_NONE       // GLFW including OpenGL headers causes ambiguity or multiple definition errors.
#include <glbinding/glbinding.h>// Initialize with glbinding::initialize()
#include <glbinding/gl/gl.h>
using namespace gl;
#else
#include IMGUI_IMPL_OPENGL_LOADER_CUSTOM
#endif
using namespace std;

static int window_w = 1024;
static int window_h = 768;


const glm::mat4 ID(1);
const glm::vec3 EX(1,0,0),
                EY(0,1,0),
                EZ(0,0,1);


class Client {
  public:
    Client(SDL_Window *w);
    ~Client(void);
    void draw(double dt);
    void keypress(SDL_Keycode kc);
    void resize(int W, int H);
    void mousedrag(int x, int y, bool left_button);
    void mouseclick(int x, int y, bool left_button);
    void mousescroll(int yoffset);
    glm::vec3 unProject(const glm::vec3& pos, const glm::mat4& modelviewproj, const glm::vec4& viewport);
  private:
    SDL_Window *window;
    GLint program;
    GLuint vao,
           vbos[3];
    glm::mat4 VP;
    bool cpu_load;
    Flocker flock;
    std::vector<Boid> boids;
    glm::vec3 cursor_pos;
    ArcballCamera camera;
    bool show_tooltips = false;
    int last_mouse_x, last_mouse_y;
    int separation_type;
    
};


/////////////////////////////////////////////////////////////////
const char *vertex_shader_text = R"blah(
  #version 130
  in vec3 position;
  in vec3 normal;
  uniform mat4 VP_matrix;
  uniform mat4 model_matrix;
  uniform mat4 normal_matrix;
  flat out vec3 world_normal;
  void main() {
    gl_Position = VP_matrix * model_matrix * vec4(position,1);
    world_normal = vec3(normal_matrix * vec4(normal,0));
  }
)blah";


const char *fragment_shader_text = R"blah(
  #version 130
  uniform vec3 light_direction;
  uniform vec3 diffuse_color;
  flat in vec3 world_normal;
  out vec4 frag_color;
  void main(void) {
    vec3 m = normalize(world_normal);
    float ml = max(0.0,dot(m,light_direction));
    vec3 color = ml * diffuse_color;
    frag_color = vec4(color,1);
  }
)blah";


glm::vec3 Client::unProject(const glm::vec3& pos, const glm::mat4& modelviewproj, const glm::vec4& viewport)
{
    glm::mat4 inv = inverse(modelviewproj);
    glm::vec4 temp = glm::vec4(pos, 1.0f);
    temp.x = ((temp.x - viewport.x) / viewport.z) * 2.0f - 1.0f;
    temp.y = ((temp.y - viewport.y) / viewport.w) * 2.0f - 1.0f;
    temp.z = 2.0f * pos.z - 1.0f;
    temp.y = -temp.y;
    temp.w = 1.0;

    glm::vec4 out = inv * temp;
    if (out.w == 0.0)
        return glm::vec3(0, 0, 0);
    out.w = 1.0 / out.w; // perspective divide

    float objCoordX = out.x * out.w;
    float objCoordY = out.y * out.w;
    float objCoordZ = out.z * out.w;

    return glm::vec3(objCoordX, objCoordY, objCoordZ);
}


Client::Client(SDL_Window *w)
    : window(w), cursor_pos(0), separation_type(3){

    camera = ArcballCamera(glm::vec3(0, 0, 8), glm::vec3(0), glm::vec3(0, 1, 0));
    // shader program
    program = glCreateProgram();
    GLenum type[2] = { GL_VERTEX_SHADER, GL_FRAGMENT_SHADER };
    const char *source[2] = { vertex_shader_text, fragment_shader_text };
    GLuint shader[2];
    for (int i=0; i < 2; ++i) {
    shader[i] = glCreateShader(type[i]);
    glShaderSource(shader[i],1,source+i,0);
    glCompileShader(shader[i]);
    GLint value;
    glGetShaderiv(shader[i],GL_COMPILE_STATUS,&value);
    if (!value) {
        char buffer[1024];
        glGetShaderInfoLog(shader[i],1024,0,buffer);
        cerr << "shader " << i << " error:" << endl;
        cerr << buffer << endl;
    }
    glAttachShader(program,shader[i]);
    }
    glLinkProgram(program);
    glDeleteShader(shader[0]);
    glDeleteShader(shader[1]);

    // mesh
    glGenVertexArrays(1,&vao);
    glGenBuffers(3,vbos);
    glBindVertexArray(vao);
    // (1) vertices
    glBindBuffer(GL_ARRAY_BUFFER,vbos[0]);
    glm::vec3 verts[5] = { {1,1,1}, {-1,1,1}, {-1,-1,1}, {1,-1,1}, {0,0,-1} };
    glBufferData(GL_ARRAY_BUFFER,sizeof(verts),verts,GL_STATIC_DRAW);
    GLint loc = glGetAttribLocation(program,"position");
    glVertexAttribPointer(loc,3,GL_FLOAT,false,sizeof(glm::vec3),0);
    glEnableVertexAttribArray(loc);
    // (2) normals
    glBindBuffer(GL_ARRAY_BUFFER,vbos[1]);
    glm::vec3 norms[5] = { {0,0,1}, {0,2,-1}, {-2,0,-1}, {0,-2,-1}, {2,0,-1} };
    for (int i=0; i < 5; ++i)
    norms[i] = glm::normalize(norms[i]);
    glBufferData(GL_ARRAY_BUFFER,sizeof(norms),norms,GL_STATIC_DRAW);
    loc = glGetAttribLocation(program,"normal");
    glVertexAttribPointer(loc,3,GL_FLOAT,false,sizeof(glm::vec3),0);
    glEnableVertexAttribArray(loc);
    // (3) faces
    glBindBuffer(GL_ELEMENT_ARRAY_BUFFER,vbos[2]);
    unsigned faces[6*3] = { 1,2,0, 2,3,0, 0,4,1, 1,4,2, 2,4,3, 0,3,4 };
    glBufferData(GL_ELEMENT_ARRAY_BUFFER,sizeof(faces),faces,GL_STATIC_DRAW);
    glBindVertexArray(0);

    // misc
    glEnable(GL_DEPTH_TEST);
    resize(window_w, window_h);
    glUseProgram(program);
    loc = glGetUniformLocation(program,"light_direction");
    glUniform3f(loc,0,0,1);
    cpu_load = false;

    // create our flock
    boids.push_back(Boid(glm::vec3(1, 0, 0), glm::vec3(1, 0, 0)));
    boids.push_back(Boid(glm::vec3(2.5, 0, 0), glm::vec3(1, 1, 0)));
    boids.push_back(Boid(glm::vec3(1, 1.5, 0), glm::vec3(0, 1, 0)));
    boids.push_back(Boid(glm::vec3(4, 4, 0), glm::vec3(1, 0, 0)));
    // add a steering target
    flock = Flocker(&boids);
    cursor_pos = glm::vec3(-0.18, -0.35, 0.2);
    flock.SteeringTargets.push_back(cursor_pos);
    // add an obstacle sphere
    flock.CollisionRadius = 2.0f;
    flock.CollisionCenter = glm::vec3(-3, -3, 0);

}


/////////////////////////////////////////////////////////////////
Client::~Client(void) {
  glUseProgram(0);
  glDeleteProgram(program);
  glDeleteVertexArrays(1,&vao);
  glDeleteBuffers(3,vbos);
}


/////////////////////////////////////////////////////////////////
void Client::draw(double dt) {
  glClearColor(1,1,1,1);
  glClearDepth(1);
  glClear(GL_COLOR_BUFFER_BIT|GL_DEPTH_BUFFER_BIT);

  glUseProgram(program);
  VP = glm::perspective(glm::radians(90.0f), float(window_w) / float(window_h), 0.1f, 150.0f)
      * camera.transform();
  GLint loc = glGetUniformLocation(program, "VP_matrix");
  glUniformMatrix4fv(loc, 1, false, &VP[0][0]);

  GLint umodel_matrix = glGetUniformLocation(program,"model_matrix"),
        unormal_matrix = glGetUniformLocation(program,"normal_matrix");
  glUniform3f(glGetUniformLocation(program,"diffuse_color"),1,0,1);

  glBindVertexArray(vao);

  for (Boid& boid : boids) {
      // orient mesh in direction of particle velocity,
      //   and parallel to plane of motion
      glm::vec3 w = -glm::normalize(boid.velocity),
          u = glm::normalize(glm::cross(w, boid.motion_normal)),
          v = glm::cross(u, w);
      glm::mat4 M = glm::translate(ID, boid.position)
          * glm::mat4(glm::mat3(u, v, w))
          * glm::scale(ID, 0.25f*boid.size*glm::vec3(1, 1, 2)),
          N = glm::mat4(glm::mat3(M));
      glUniformMatrix4fv(umodel_matrix, 1, false, &M[0][0]);
      glUniformMatrix4fv(unormal_matrix, 1, false, &N[0][0]);
      glDrawElements(GL_TRIANGLES, 6 * 3, GL_UNSIGNED_INT, 0);
  }

  // draw world target cube
  glm::mat4 model = glm::mat4(1.0f);
  model = glm::translate(model, cursor_pos);
  model = glm::scale(model, glm::vec3(0.3f, 0.3f, 0.3f));
  glUseProgram(program);
  glUniformMatrix4fv(umodel_matrix, 1, false, &model[0][0]);
  renderCube();

  // draw world collision sphere
  model = glm::mat4(1.0f);
  model = glm::translate(model, flock.CollisionCenter);
  model = glm::scale(model, glm::vec3(flock.CollisionRadius, flock.CollisionRadius, flock.CollisionRadius));
  glUseProgram(program);
  glUniformMatrix4fv(umodel_matrix, 1, false, &model[0][0]);
  renderSphere();


  glBindVertexArray(0);

  // start the Dear ImGui frame
  ImGui_ImplOpenGL3_NewFrame();
  ImGui_ImplSDL2_NewFrame(window);
  ImGui::NewFrame();
  {
      static float f = 0.0f;
      static int counter = 0;
      ImGui::Begin("Controls");                          // Create a window called "Controls" and append into it.
      if (ImGui::CollapsingHeader("Agent Settings")) {
          ImGui::Checkbox("Show tooltips", &show_tooltips);

          ImGui::SliderFloat("Perception radius", &flock.PerceptionRadius, 1.0f, 40.0f, "%.3f");
          if (show_tooltips && ImGui::IsItemHovered())
              ImGui::SetTooltip("Perception refers to the vision of each boid.Only boids within this distance influence each other");

          ImGui::SliderFloat("Separation weight", &flock.SeparationWeight, 0.1f, 5.0f, "%.3f");
          if (show_tooltips && ImGui::IsItemHovered())
              ImGui::SetTooltip("How much agents repel each other");
          const char* separationType[] = { "LINEAR", "INVERSE LINEAR", "QUADRATIC", "INVERSE QUADRATIC"};
          if (ImGui::Combo("Separation function", &separation_type, separationType, IM_ARRAYSIZE(separationType))) {
              switch (separation_type) {
              case 0:
                  flock.SeparationType = DistanceType::LINEAR;
                  break;
              case 1:
                  flock.SeparationType = DistanceType::INVERSE_LINEAR;
                  break;
              case 2:
                  flock.SeparationType = DistanceType::QUADRATIC;
                  break;
              case 3:
                  flock.SeparationType = DistanceType::INVERSE_QUADRATIC;
                  break;
              default:
                  break;
              }
          }
          if (show_tooltips && ImGui::IsItemHovered())
              ImGui::SetTooltip("Function that controls the rate of separation between agents");


          ImGui::SliderFloat("Alignment weight", &flock.AlignmentWeight, 0.1f, 5.0f, "%.3f");
          if (show_tooltips && ImGui::IsItemHovered())
              ImGui::SetTooltip("How much agent's velocity matches its neighboring agents");

          ImGui::SliderFloat("Cohesion weight", &flock.CohesionWeight, 0.1f, 5.0f, "%.3f");
          if (show_tooltips && ImGui::IsItemHovered())
              ImGui::SetTooltip("How much agent should stay close to its neighboring agents");

          ImGui::SliderFloat("Steering weight", &flock.SteeringWeight, 0.1f, 10.0f, "%.3f");
          if (show_tooltips && ImGui::IsItemHovered())
              ImGui::SetTooltip("How much agents should home in on a target location");

          ImGui::SliderFloat("Max acceleration", &flock.MaxAcceleration, 1.0f, 10.0f, "%.3f");
          ImGui::SliderFloat("Max velocity", &flock.MaxVelocity, 1.0f, 20.0f, "%.3f");
      }

      ImGui::Text("Agents in scene = %i", boids.size()); ImGui::SameLine(200);
      if (ImGui::Button("Add Agent"))
      {
          // create a random agent
          float random_x = cursor_pos.x + random_double(-2.0, 2.0);
          float random_y = cursor_pos.y + random_double(-2.0, 2.0);
          float random_z = cursor_pos.z + random_double(-2.0, 2.0);
          boids.push_back(Boid(glm::vec3(random_x, random_y, random_z), 
              glm::vec3(random_double(-1.0, 1.0), random_double(-1.0, 1.0), random_double(-1.0, 1.0))));
      }
      ImGui::Text("World target position (%.3f, %.3f, %.3f)", cursor_pos.x, cursor_pos.y, cursor_pos.z);
      ImGui::Text("Camera position (%.3f, %.3f, %.3f)", camera.eye().x, camera.eye().y, camera.eye().z);
      ImGui::Text("-------------------------------------------------");
      ImGui::Text("LEFT MOUSE BUTTON + drag to rotate camera about world origin");
      ImGui::Text("MOUSE WHEEL to zoom in/out");
      ImGui::Text("RIGHT MOUSE BUTTON click to re-position agent target position (cube)");
      ImGui::Text("RIGHT MOUSE BUTTON + drag to pan the camera");

      ImGui::Text("Application average %.3f ms/frame (%.1f FPS)", 1000.0f / ImGui::GetIO().Framerate, ImGui::GetIO().Framerate);

      ImGui::End();

  }
  // rendering
  ImGui::Render();
  ImGui_ImplOpenGL3_RenderDrawData(ImGui::GetDrawData());

  if (cpu_load)
    this_thread::sleep_for(chrono::milliseconds(100));

  // update our agent-based simulation
  flock.update(dt);
}


void Client::keypress(SDL_Keycode kc) {
  if (kc == SDLK_SPACE) {
    cpu_load = !cpu_load;
    SDL_SetWindowTitle(window,cpu_load ? "CPU load on"
                                       : "CPU load off");
  }
}


void Client::resize(int W, int H) {
  glViewport(0,0,W,H);

  VP = glm::perspective(glm::radians(90.0f), float(W) / float(H), 0.1f, 100.0f)
      * camera.transform();
  glUseProgram(program);
  GLint loc = glGetUniformLocation(program,"VP_matrix");
  glUniformMatrix4fv(loc,1,false,&VP[0][0]);
}


void Client::mousedrag(int x, int y, bool left_button) {

    ImGuiIO& io = ImGui::GetIO();
    if (left_button && !io.WantCaptureMouse) {
        float prevMouseX = 2.0f * last_mouse_x / window_w - 1;
        float prevMouseY = -1.0f * (2.0f * last_mouse_y / window_h - 1);
        float curMouseX = 2.0f * x / window_w - 1;
        float curMouseY = -1.0f * (2.0f * y / window_h - 1);
        camera.rotate(glm::vec2(prevMouseX, prevMouseY), glm::vec2(curMouseX, curMouseY));
    }
    else if (!left_button && !io.WantCaptureMouse) {
        float prevMouseX = 2.0f * last_mouse_x / window_w - 1;
        float prevMouseY = -1.0f * (2.0f * last_mouse_y / window_h - 1);
        float curMouseX = 2.0f * x / window_w - 1;
        float curMouseY = -1.0f * (2.0f * y / window_h - 1);
        glm::vec2 mouse_delta = glm::vec2(curMouseX - prevMouseX, curMouseY - prevMouseY);
        camera.pan(mouse_delta);
    }

    last_mouse_x = x;
    last_mouse_y = y;
}

void Client::mouseclick(int x, int y, bool left_button) {

    ImGuiIO& io = ImGui::GetIO();
    if (!io.WantCaptureMouse && !left_button) {
        glm::vec3 near_plane = glm::vec3(float(x), float(y), 0.0f);
        glm::vec3 world_pos = unProject(near_plane, VP, glm::vec4(0, 0, window_w, window_h));
        // place the cursor point on a plane perpendicular to camera view and close to world origin
        glm::vec3 pick_dir = glm::normalize(world_pos - camera.eye());
        glm::vec3 planeN = -camera.dir();
        glm::vec3 wall_point(random_double(-0.5, 0.5), random_double(-0.5, 0.5), random_double(-0.5, 0.5));
        float t = glm::dot(wall_point - camera.eye(), planeN) / glm::dot(pick_dir, planeN);
        cursor_pos = camera.eye() + pick_dir * t;

        if (flock.SteeringTargets.empty()) {
            flock.SteeringTargets.push_back(cursor_pos);
        }
        flock.SteeringTargets[0] = cursor_pos;
    }
    last_mouse_x = x;
    last_mouse_y = y;
}

void Client::mousescroll(int yoffset) {
    ImGuiIO& io = ImGui::GetIO();
    if (io.WantCaptureMouse) return;

    float distanceSq = glm::distance2(camera.center(), camera.eye());
    if (distanceSq < 900 && yoffset < 0) {
        // zoom out
        camera.zoom(yoffset);
    }
    else if (yoffset > 0){
        // zoom in
        camera.zoom(yoffset);
    }
}


/////////////////////////////////////////////////////////////////
//
/////////////////////////////////////////////////////////////////
int main(int argc, char *argv[]) {

  // SDL: initialize and create a window
  SDL_Init(SDL_INIT_VIDEO);
  const char *title = "CS 561 Project 1 [Agent-based simulation]";
  SDL_Window *window = SDL_CreateWindow(title,SDL_WINDOWPOS_UNDEFINED,
                                        SDL_WINDOWPOS_UNDEFINED,window_w,window_h,
                                        SDL_WINDOW_OPENGL|SDL_WINDOW_RESIZABLE);
  SDL_GLContext context = SDL_GL_CreateContext(window);

  // GLEW: get function bindings (if possible)
  GLenum value = glewInit();
  if (value != GLEW_OK) {
    cout << glewGetErrorString(value) << endl;
    SDL_GL_DeleteContext(context);
    SDL_Quit();
    return -1;
  }

  // Setup Dear ImGui context
  IMGUI_CHECKVERSION();
  ImGui::CreateContext();
  ImGuiIO& io = ImGui::GetIO(); (void)io;

  // Setup Dear ImGui style
  ImGui::StyleColorsDark();

  std::string glsl_version = "#version 130";

  // setup platform/renderer bindings
  ImGui_ImplSDL2_InitForOpenGL(window, context);
  ImGui_ImplOpenGL3_Init(glsl_version.c_str());

  // animation loop
  bool done = false;
  Client *client = new Client(window);
  Uint32 ticks_last = SDL_GetTicks();
  while (!done) {
    SDL_Event event;
    while (SDL_PollEvent(&event)) {
      switch (event.type) {
        case SDL_QUIT:
          done = true;
          break;
        case SDL_KEYDOWN:
          if (event.key.keysym.sym == SDLK_ESCAPE)
            done = true;
          else
            client->keypress(event.key.keysym.sym);
          break;
        case SDL_WINDOWEVENT:
          if (event.window.event == SDL_WINDOWEVENT_RESIZED)
            client->resize(event.window.data1,event.window.data2);
          break;
        case SDL_MOUSEMOTION:
          if ((event.motion.state&SDL_BUTTON_LMASK) != 0
              || (event.motion.state&SDL_BUTTON_RMASK) != 0)
            client->mousedrag(event.motion.x,event.motion.y,
                event.button.button == SDL_BUTTON_LEFT);
          break;
        case SDL_MOUSEBUTTONDOWN:
            switch (event.button.button) {
            case SDL_BUTTON_LEFT: {
                client->mouseclick(event.motion.x, event.motion.y, true);
                }
                break;

            case SDL_BUTTON_RIGHT: {
                client->mouseclick(event.motion.x, event.motion.y, false);
                }
                break;
            }
            break;
        case SDL_MOUSEBUTTONUP:
            client->mouseclick(event.motion.x, event.motion.y, true);
            break;
        case SDL_MOUSEWHEEL:
            client->mousescroll(event.wheel.y);
            break;
      }
    }
    Uint32 ticks = SDL_GetTicks();
    double dt = 0.001*(ticks - ticks_last);
    ticks_last = ticks;
    client->draw(dt);
    SDL_GL_SwapWindow(window);
  }

  ImGui_ImplOpenGL3_Shutdown();
  ImGui_ImplSDL2_Shutdown();
  ImGui::DestroyContext();

  // clean up
  delete client;
  SDL_GL_DeleteContext(context);
  SDL_Quit();
  return 0;
}



