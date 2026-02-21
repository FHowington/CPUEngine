#pragma once

#include "overlay.h"
#include <functional>
#include <string>
#include <vector>

// ─── Menu item types ─────────────────────────────────────────────────────────

enum class MenuItemType { Label, Toggle, Slider, Action, Submenu };

struct MenuItem {
  std::string text;
  MenuItemType type = MenuItemType::Label;

  // Toggle
  bool* toggleVal = nullptr;

  // Slider
  float* sliderVal = nullptr;
  float sliderMin = 0;
  float sliderMax = 1;
  float sliderStep = 0.1f;

  // Action
  std::function<void()> action;

  // Submenu
  struct Menu* submenu = nullptr;

  // Helpers to build items
  static MenuItem label(const std::string& t) {
    return {t, MenuItemType::Label};
  }
  static MenuItem toggle(const std::string& t, bool* val) {
    MenuItem m{t, MenuItemType::Toggle};
    m.toggleVal = val;
    return m;
  }
  static MenuItem slider(const std::string& t, float* val, float lo, float hi, float step = 0.1f) {
    MenuItem m{t, MenuItemType::Slider};
    m.sliderVal = val;
    m.sliderMin = lo;
    m.sliderMax = hi;
    m.sliderStep = step;
    return m;
  }
  static MenuItem actionItem(const std::string& t, std::function<void()> fn) {
    MenuItem m{t, MenuItemType::Action};
    m.action = std::move(fn);
    return m;
  }
  static MenuItem sub(const std::string& t, Menu* child) {
    MenuItem m{t, MenuItemType::Submenu};
    m.submenu = child;
    return m;
  }

  bool selectable() const { return type != MenuItemType::Label; }
};

// ─── Menu ────────────────────────────────────────────────────────────────────

struct Menu {
  std::string title;
  std::vector<MenuItem> items;
  int cursor = 0;       // index of highlighted item
  Menu* parent = nullptr;
  bool visible = false;

  Menu() = default;
  Menu(const std::string& t) : title(t) {}

  void addItem(MenuItem item) {
    if (item.type == MenuItemType::Submenu && item.submenu)
      item.submenu->parent = this;
    items.push_back(std::move(item));
  }

  // Move cursor to next/prev selectable item
  void cursorDown() {
    int n = (int)items.size();
    for (int i = 1; i < n; ++i) {
      int idx = (cursor + i) % n;
      if (items[idx].selectable()) { cursor = idx; return; }
    }
  }
  void cursorUp() {
    int n = (int)items.size();
    for (int i = 1; i < n; ++i) {
      int idx = (cursor - i + n) % n;
      if (items[idx].selectable()) { cursor = idx; return; }
    }
  }

  // Returns submenu pointer if a submenu was entered, nullptr otherwise
  Menu* activate() {
    if (cursor < 0 || cursor >= (int)items.size()) return nullptr;
    MenuItem& item = items[cursor];
    switch (item.type) {
      case MenuItemType::Toggle:
        if (item.toggleVal) *item.toggleVal = !*item.toggleVal;
        break;
      case MenuItemType::Action:
        if (item.action) item.action();
        break;
      case MenuItemType::Submenu:
        if (item.submenu) { item.submenu->visible = true; return item.submenu; }
        break;
      case MenuItemType::Slider:
        // Enter/activate on slider does nothing; use left/right
        break;
      default: break;
    }
    return nullptr;
  }

  void adjustSlider(int dir) {
    if (cursor < 0 || cursor >= (int)items.size()) return;
    MenuItem& item = items[cursor];
    if (item.type == MenuItemType::Slider && item.sliderVal) {
      *item.sliderVal += dir * item.sliderStep;
      if (*item.sliderVal < item.sliderMin) *item.sliderVal = item.sliderMin;
      if (*item.sliderVal > item.sliderMax) *item.sliderVal = item.sliderMax;
    }
  }

  // ── Drawing ────────────────────────────────────────────────────────────

  void draw(int originX, int originY, int scale = 2) const {
    if (!visible) return;

    constexpr int PAD = 8;
    const int charW = 8 * scale;
    const int lineH = 8 * scale + 2;

    // Measure width from longest line
    int maxChars = (int)title.size();
    for (auto& item : items) {
      int len = (int)item.text.size() + 4; // room for value suffix
      if (item.type == MenuItemType::Toggle) len += 6;
      if (item.type == MenuItemType::Slider) len += 10;
      if (item.type == MenuItemType::Submenu) len += 2;
      if (len > maxChars) maxChars = len;
    }

    const int pw = maxChars * charW + PAD * 2;
    const int ph = ((int)items.size() + 2) * lineH + PAD * 2; // +2 for title + separator

    Overlay::fillRect(originX, originY, pw, ph, 0x1A1A2E, 220);
    Overlay::drawRect(originX, originY, pw, ph, 0x4A4A6A);

    int tx = originX + PAD;
    int ty = originY + PAD;

    // Title
    Overlay::drawText(tx, ty, title, 0xFFDD00, scale);
    ty += lineH;
    // Separator line
    Overlay::fillRect(tx, ty + lineH / 2 - 1, pw - PAD * 2, 1, 0x4A4A6A, 255);
    ty += lineH;

    char buf[64];
    for (int i = 0; i < (int)items.size(); ++i) {
      const MenuItem& item = items[i];
      bool selected = (i == cursor);
      unsigned color = 0xCCCCCC;

      if (selected) {
        Overlay::fillRect(originX + 2, ty - 1, pw - 4, lineH, 0x3A3A5E, 200);
        color = 0xFFFFFF;
      }

      switch (item.type) {
        case MenuItemType::Label:
          Overlay::drawText(tx, ty, item.text, 0x888888, scale);
          break;
        case MenuItemType::Toggle:
          snprintf(buf, sizeof(buf), "%s: %s", item.text.c_str(),
                   (item.toggleVal && *item.toggleVal) ? "ON" : "OFF");
          Overlay::drawText(tx, ty, buf,
                            (item.toggleVal && *item.toggleVal) ? 0x55FF55 : color, scale);
          break;
        case MenuItemType::Slider:
          snprintf(buf, sizeof(buf), "%s: %.1f", item.text.c_str(),
                   item.sliderVal ? *item.sliderVal : 0.0f);
          Overlay::drawText(tx, ty, buf, 0x88FFFF, scale);
          break;
        case MenuItemType::Action:
          Overlay::drawText(tx, ty, item.text, color, scale);
          break;
        case MenuItemType::Submenu:
          snprintf(buf, sizeof(buf), "%s >", item.text.c_str());
          Overlay::drawText(tx, ty, buf, 0xAAAAFF, scale);
          break;
      }

      if (selected) {
        Overlay::drawText(tx - charW, ty, ">", 0xFFDD00, scale);
      }

      ty += lineH;
    }
  }
};

// ─── MenuStack — manages open menu hierarchy and input ───────────────────────

class MenuStack {
 public:
  // Set the root menu. Does not open it.
  void setRoot(Menu* root) { _root = root; }

  bool isOpen() const { return _current && _current->visible; }

  void open() {
    if (_root) { _root->visible = true; _root->cursor = 0; _current = _root; }
  }

  void close() {
    // Close entire stack
    for (Menu* m = _current; m; m = m->parent) m->visible = false;
    _current = nullptr;
  }

  // Returns true if the key was consumed by the menu
  bool handleKey(SDL_Keycode key) {
    if (!isOpen()) return false;

    switch (key) {
      case SDLK_UP:     _current->cursorUp();   return true;
      case SDLK_DOWN:   _current->cursorDown();  return true;
      case SDLK_LEFT:
        _current->adjustSlider(-1);
        return true;
      case SDLK_RIGHT:
        _current->adjustSlider(1);
        return true;
      case SDLK_RETURN:
      case SDLK_SPACE: {
        Menu* sub = _current->activate();
        if (sub) _current = sub;
        return true;
      }
      case SDLK_ESCAPE:
      case SDLK_BACKSPACE:
        if (_current->parent) {
          _current->visible = false;
          _current = _current->parent;
        } else {
          close();
        }
        return true;
      default:
        return false;
    }
  }

  void draw(int x = 12, int y = 12, int scale = 2) const {
    if (_current) _current->draw(x, y, scale);
  }

 private:
  Menu* _root = nullptr;
  Menu* _current = nullptr;
};
