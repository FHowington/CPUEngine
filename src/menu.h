#pragma once

#include "Window.h"
#include "overlay.h"
#include <functional>
#include <string>
#include <vector>

// ─── Menu item types ─────────────────────────────────────────────────────────

enum class MenuItemType { Label, Separator, Toggle, Slider, Action, Submenu, Back };

struct MenuItem {
  std::string text;
  MenuItemType type = MenuItemType::Label;
  bool* toggleVal = nullptr;
  float* sliderVal = nullptr;
  float sliderMin = 0, sliderMax = 1, sliderStep = 0.1f;
  std::function<void()> action;
  struct Menu* submenu = nullptr;

  bool selectable() const {
    return type != MenuItemType::Label && type != MenuItemType::Separator;
  }

  static MenuItem label(const std::string& t) { return {t, MenuItemType::Label}; }
  static MenuItem separator() { return {"", MenuItemType::Separator}; }
  static MenuItem back() { return {"< Back", MenuItemType::Back}; }

  static MenuItem toggle(const std::string& t, bool* val) {
    MenuItem m{t, MenuItemType::Toggle}; m.toggleVal = val; return m;
  }
  static MenuItem slider(const std::string& t, float* val, float lo, float hi, float step = 0.1f) {
    MenuItem m{t, MenuItemType::Slider};
    m.sliderVal = val; m.sliderMin = lo; m.sliderMax = hi; m.sliderStep = step;
    return m;
  }
  static MenuItem actionItem(const std::string& t, std::function<void()> fn) {
    MenuItem m{t, MenuItemType::Action}; m.action = std::move(fn); return m;
  }
  static MenuItem sub(const std::string& t, Menu* child) {
    MenuItem m{t, MenuItemType::Submenu}; m.submenu = child; return m;
  }
};

// ─── Menu ────────────────────────────────────────────────────────────────────

struct Menu {
  std::string title;
  std::vector<MenuItem> items;
  int cursor = 0;
  Menu* parent = nullptr;

  Menu() = default;
  Menu(const std::string& t) : title(t) {}

  void addItem(MenuItem item) {
    if (item.type == MenuItemType::Submenu && item.submenu)
      item.submenu->parent = this;
    items.push_back(std::move(item));
  }

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
        if (item.submenu) return item.submenu;
        break;
      case MenuItemType::Back:
        return parent; // signal to go back
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
};

// ─── MenuStyle ───────────────────────────────────────────────────────────────

struct MenuStyle {
  int scale      = 2;
  int pad        = 8;
  int minCols    = 24;
  unsigned bgColor     = 0x1A1A2E;
  unsigned char bgAlpha = 220;
  unsigned borderColor = 0x4A4A6A;
  unsigned titleColor  = 0xFFDD00;
  unsigned labelColor  = 0x888888;
  unsigned textColor   = 0xCCCCCC;
  unsigned selColor    = 0xFFFFFF;
  unsigned selBg       = 0x3A3A5E;
  unsigned toggleOn    = 0x55FF55;
  unsigned sliderColor = 0x88FFFF;
  unsigned sliderTrack = 0x333355;
  unsigned sliderFill  = 0x4488AA;
  unsigned sliderKnob  = 0xFFFFFF;
  unsigned subColor    = 0xAAAAFF;
  unsigned backColor   = 0xAAAA88;
  unsigned cursorColor = 0xFFDD00;
};

// ─── Hit rect for mouse interaction ─────────────────────────────────────────

struct ItemRect {
  int x, y, w, h;
  int itemIdx;
  // Slider track region (within the item rect)
  int trackX, trackW;
};

// ─── MenuStack — manages open menu hierarchy, input, and rendering ───────────

class MenuStack {
 public:
  void setRoot(Menu* root) { _root = root; }
  void setPosition(int x, int y) { _x = x; _y = y; }
  void setStyle(const MenuStyle& s) { _style = s; }

  bool isOpen() const { return _current != nullptr; }

  void open() {
    if (_root) { _current = _root; _current->cursor = 0; snapCursor(); }
  }
  void close() { _current = nullptr; _dragging = false; }
  void toggle() { isOpen() ? close() : open(); }

  bool handleKey(SDL_Keycode key) {
    if (!isOpen()) return false;
    switch (key) {
      case SDLK_UP:    _current->cursorUp();   return true;
      case SDLK_DOWN:  _current->cursorDown();  return true;
      case SDLK_LEFT:  _current->adjustSlider(-1); return true;
      case SDLK_RIGHT: _current->adjustSlider(1);  return true;
      case SDLK_RETURN:
      case SDLK_SPACE: {
        Menu* result = _current->activate();
        if (result) {
          if (result == _current->parent) {
            // Back
            _current = result;
          } else {
            _current = result;
            snapCursor();
          }
        }
        return true;
      }
      case SDLK_ESCAPE:
      case SDLK_BACKSPACE:
        if (_current->parent) _current = _current->parent;
        else close();
        return true;
      default: return false;
    }
  }

  bool handleMouse(const SDL_Event& ev) {
    if (!isOpen()) return false;

    int mx = 0, my = 0;
    Uint32 winID = 0;
    if (ev.type == SDL_MOUSEBUTTONDOWN || ev.type == SDL_MOUSEBUTTONUP) {
      mx = ev.button.x; my = ev.button.y; winID = ev.button.windowID;
    } else if (ev.type == SDL_MOUSEMOTION) {
      mx = ev.motion.x; my = ev.motion.y; winID = ev.motion.windowID;
    } else {
      return false;
    }

    // Map window coords → framebuffer coords (W×H)
    SDL_Window* win = SDL_GetWindowFromID(winID);
    if (win) {
      int ww, wh;
      SDL_GetWindowSize(win, &ww, &wh);
      if (ww > 0 && wh > 0) {
        mx = mx * (int)W / ww;
        my = my * (int)H / wh;
      }
    }

    if (ev.type == SDL_MOUSEBUTTONDOWN && ev.button.button == SDL_BUTTON_LEFT) {
      for (auto& r : _hitRects) {
        if (mx >= r.x && mx < r.x + r.w && my >= r.y && my < r.y + r.h) {
          auto& item = _current->items[r.itemIdx];
          _current->cursor = r.itemIdx;

          if (item.type == MenuItemType::Slider && item.sliderVal && r.trackW > 0) {
            _dragging = true;
            _dragIdx = r.itemIdx;
            setSliderFromMouse(item, r, mx);
          } else if (item.type == MenuItemType::Toggle) {
            _current->activate();
          } else if (item.type == MenuItemType::Submenu) {
            Menu* sub = _current->activate();
            if (sub) { _current = sub; snapCursor(); }
          } else if (item.type == MenuItemType::Back) {
            if (_current->parent) _current = _current->parent;
          } else if (item.type == MenuItemType::Action) {
            _current->activate();
          }
          return true;
        }
      }
      return false;
    }

    if (ev.type == SDL_MOUSEMOTION && _dragging) {
      for (auto& r : _hitRects) {
        if (r.itemIdx == _dragIdx) {
          auto& item = _current->items[r.itemIdx];
          setSliderFromMouse(item, r, mx);
          return true;
        }
      }
    }

    if (ev.type == SDL_MOUSEBUTTONUP && ev.button.button == SDL_BUTTON_LEFT) {
      _dragging = false;
      return true;
    }

    return false;
  }

  void draw() {
    if (!_current) return;
    _hitRects.clear();
    drawMenu(*_current, _x, _y);
  }

 private:
  Menu* _root = nullptr;
  Menu* _current = nullptr;
  int _x = 12, _y = 12;
  MenuStyle _style;
  std::vector<ItemRect> _hitRects;
  bool _dragging = false;
  int _dragIdx = -1;

  void snapCursor() {
    if (_current && !_current->items.empty() && !_current->items[_current->cursor].selectable())
      _current->cursorDown();
  }

  void setSliderFromMouse(MenuItem& item, const ItemRect& r, int mx) {
    if (!item.sliderVal || r.trackW <= 0) return;
    float t = (float)(mx - r.trackX) / (float)r.trackW;
    if (t < 0) t = 0; if (t > 1) t = 1;
    float raw = item.sliderMin + t * (item.sliderMax - item.sliderMin);
    // Snap to step
    float steps = (raw - item.sliderMin) / item.sliderStep;
    *item.sliderVal = item.sliderMin + roundf(steps) * item.sliderStep;
    if (*item.sliderVal < item.sliderMin) *item.sliderVal = item.sliderMin;
    if (*item.sliderVal > item.sliderMax) *item.sliderVal = item.sliderMax;
  }

  void drawMenu(const Menu& menu, int ox, int oy) {
    const int S = _style.scale;
    const int PAD = _style.pad;
    const int charW = 8 * S;
    const int lineH = 8 * S + 4;

    // Measure width
    int maxChars = std::max((int)menu.title.size(), _style.minCols);
    for (auto& item : menu.items) {
      int len = (int)item.text.size();
      if (item.type == MenuItemType::Toggle) len += 6;
      else if (item.type == MenuItemType::Slider) len += 10;
      else if (item.type == MenuItemType::Submenu) len += 2;
      if (len > maxChars) maxChars = len;
    }

    int totalH = 2 * lineH;
    for (auto& item : menu.items)
      totalH += (item.type == MenuItemType::Separator) ? lineH / 2 : lineH;
    // Extra row for slider track below label
    for (auto& item : menu.items)
      if (item.type == MenuItemType::Slider) totalH += lineH / 2 + 2;

    const int pw = maxChars * charW + PAD * 2;
    const int ph = totalH + PAD * 2;

    Overlay::fillRect(ox, oy, pw, ph, _style.bgColor, _style.bgAlpha);
    Overlay::drawRect(ox, oy, pw, ph, _style.borderColor);

    int tx = ox + PAD;
    int ty = oy + PAD;

    // Title
    Overlay::drawText(tx, ty, menu.title, _style.titleColor, S);
    ty += lineH;
    Overlay::fillRect(tx, ty + lineH / 2 - 1, pw - PAD * 2, 1, _style.borderColor, 255);
    ty += lineH;

    char buf[64];
    for (int i = 0; i < (int)menu.items.size(); ++i) {
      const MenuItem& item = menu.items[i];

      if (item.type == MenuItemType::Separator) {
        int sy = ty + lineH / 4 - 1;
        Overlay::fillRect(tx, sy, pw - PAD * 2, 1, _style.borderColor, 180);
        ty += lineH / 2;
        continue;
      }

      bool sel = (i == menu.cursor);
      int rowH = lineH;
      if (item.type == MenuItemType::Slider) rowH += lineH / 2 + 2;

      if (sel)
        Overlay::fillRect(ox + 2, ty - 1, pw - 4, rowH, _style.selBg, 200);

      unsigned color = sel ? _style.selColor : _style.textColor;

      // Record hit rect for this item
      ItemRect hr{ox + 2, ty - 1, pw - 4, rowH, i, 0, 0};

      switch (item.type) {
        case MenuItemType::Label:
          Overlay::drawText(tx, ty, item.text, _style.labelColor, S);
          break;

        case MenuItemType::Toggle: {
          bool on = item.toggleVal && *item.toggleVal;
          Overlay::drawText(tx, ty, item.text, color, S);
          // Draw toggle indicator on the right
          int indX = ox + pw - PAD - 6 * charW;
          Overlay::drawText(indX, ty, on ? "[ON]" : "[OFF]",
                            on ? _style.toggleOn : _style.labelColor, S);
          break;
        }

        case MenuItemType::Slider: {
          // Label + value on first line
          int prec = 1;
          if (item.sliderStep < 0.1f)  prec = 2;
          if (item.sliderStep < 0.01f) prec = 3;
          snprintf(buf, sizeof(buf), "%s: %.*f", item.text.c_str(),
                   prec, item.sliderVal ? *item.sliderVal : 0.0f);
          Overlay::drawText(tx, ty, buf, _style.sliderColor, S);

          // Slider track on second line
          int trackY = ty + lineH + 1;
          int trackH = lineH / 2;
          int trackX = tx;
          int trackW = pw - PAD * 2;

          // Track background
          Overlay::fillRect(trackX, trackY, trackW, trackH, _style.sliderTrack, 255);

          // Filled portion
          float t = 0;
          if (item.sliderVal && item.sliderMax > item.sliderMin)
            t = (*item.sliderVal - item.sliderMin) / (item.sliderMax - item.sliderMin);
          int fillW = (int)(t * trackW);
          if (fillW > 0)
            Overlay::fillRect(trackX, trackY, fillW, trackH, _style.sliderFill, 255);

          // Knob
          int knobX = trackX + fillW - 2;
          if (knobX < trackX) knobX = trackX;
          Overlay::fillRect(knobX, trackY - 1, 4, trackH + 2, _style.sliderKnob, 255);

          // Track border
          Overlay::drawRect(trackX, trackY, trackW, trackH, _style.borderColor);

          hr.trackX = trackX;
          hr.trackW = trackW;
          break;
        }

        case MenuItemType::Action:
          Overlay::drawText(tx, ty, item.text, color, S);
          break;

        case MenuItemType::Submenu:
          snprintf(buf, sizeof(buf), "%s >", item.text.c_str());
          Overlay::drawText(tx, ty, buf, _style.subColor, S);
          break;

        case MenuItemType::Back:
          Overlay::drawText(tx, ty, item.text, _style.backColor, S);
          break;

        default: break;
      }

      if (sel)
        Overlay::drawText(tx - charW, ty, ">", _style.cursorColor, S);

      _hitRects.push_back(hr);
      ty += rowH;
    }
  }
};
