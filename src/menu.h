#pragma once

#include "overlay.h"
#include <functional>
#include <string>
#include <vector>

// ─── Menu item types ─────────────────────────────────────────────────────────

enum class MenuItemType { Label, Separator, Toggle, Slider, Action, Submenu };

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

  // ── Builders ──
  static MenuItem label(const std::string& t) { return {t, MenuItemType::Label}; }
  static MenuItem separator() { return {"", MenuItemType::Separator}; }

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

  // Returns submenu if one was entered
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

// ─── MenuStack — manages open menu hierarchy, input, and rendering ───────────

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
  unsigned subColor    = 0xAAAAFF;
  unsigned cursorColor = 0xFFDD00;
};

class MenuStack {
 public:
  void setRoot(Menu* root) { _root = root; }
  void setPosition(int x, int y) { _x = x; _y = y; }
  void setStyle(const MenuStyle& s) { _style = s; }

  bool isOpen() const { return _current != nullptr; }

  void open() {
    if (_root) { _current = _root; _current->cursor = 0; snapCursor(); }
  }
  void close() { _current = nullptr; }

  void toggle() { isOpen() ? close() : open(); }

  // Returns true if the key was consumed
  bool handleKey(SDL_Keycode key) {
    if (!isOpen()) return false;
    switch (key) {
      case SDLK_UP:    _current->cursorUp();   return true;
      case SDLK_DOWN:  _current->cursorDown();  return true;
      case SDLK_LEFT:  _current->adjustSlider(-1); return true;
      case SDLK_RIGHT: _current->adjustSlider(1);  return true;
      case SDLK_RETURN:
      case SDLK_SPACE: {
        Menu* sub = _current->activate();
        if (sub) { _current = sub; snapCursor(); }
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

  void draw() const {
    if (!_current) return;
    drawMenu(*_current, _x, _y);
  }

 private:
  Menu* _root = nullptr;
  Menu* _current = nullptr;
  int _x = 12, _y = 12;
  MenuStyle _style;

  void snapCursor() {
    // Ensure cursor is on a selectable item
    if (_current && !_current->items.empty() && !_current->items[_current->cursor].selectable())
      _current->cursorDown();
  }

  void drawMenu(const Menu& menu, int ox, int oy) const {
    const int S = _style.scale;
    const int PAD = _style.pad;
    const int charW = 8 * S;
    const int lineH = 8 * S + 2;

    // Measure width
    int maxChars = std::max((int)menu.title.size(), _style.minCols);
    for (auto& item : menu.items) {
      int len = (int)item.text.size();
      if (item.type == MenuItemType::Toggle) len += 6;
      else if (item.type == MenuItemType::Slider) len += 10;
      else if (item.type == MenuItemType::Submenu) len += 2;
      if (len > maxChars) maxChars = len;
    }

    // Count visible rows (separators are half-height)
    int totalH = 2 * lineH; // title + separator
    for (auto& item : menu.items)
      totalH += (item.type == MenuItemType::Separator) ? lineH / 2 : lineH;

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
      if (sel)
        Overlay::fillRect(ox + 2, ty - 1, pw - 4, lineH, _style.selBg, 200);

      unsigned color = sel ? _style.selColor : _style.textColor;

      switch (item.type) {
        case MenuItemType::Label:
          Overlay::drawText(tx, ty, item.text, _style.labelColor, S);
          break;
        case MenuItemType::Toggle:
          snprintf(buf, sizeof(buf), "%s: %s", item.text.c_str(),
                   (item.toggleVal && *item.toggleVal) ? "ON" : "OFF");
          Overlay::drawText(tx, ty, buf,
                   (item.toggleVal && *item.toggleVal) ? _style.toggleOn : color, S);
          break;
        case MenuItemType::Slider:
          snprintf(buf, sizeof(buf), "%s: %.1f", item.text.c_str(),
                   item.sliderVal ? *item.sliderVal : 0.0f);
          Overlay::drawText(tx, ty, buf, _style.sliderColor, S);
          break;
        case MenuItemType::Action:
          Overlay::drawText(tx, ty, item.text, color, S);
          break;
        case MenuItemType::Submenu:
          snprintf(buf, sizeof(buf), "%s >", item.text.c_str());
          Overlay::drawText(tx, ty, buf, _style.subColor, S);
          break;
        default: break;
      }

      if (sel)
        Overlay::drawText(tx - charW, ty, ">", _style.cursorColor, S);

      ty += lineH;
    }
  }
};
