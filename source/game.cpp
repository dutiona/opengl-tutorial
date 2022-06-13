#include "game.hpp"

Game::Game()
    : m_is_running(false)
{
}

void Game::run()
{
  m_is_running = true;
}

void Game::quit()
{
  m_is_running = false;
}
