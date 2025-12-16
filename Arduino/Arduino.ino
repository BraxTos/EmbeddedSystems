#include <Arduino.h>

/* =========================================================
   Command language
   ========================================================= */

enum class CommandId {
    Forward,
    Backward,
    Left90,
    Right90,
    PaintOn,
    PaintOff,
    Wait
};

struct CommandSpec {
    const char* token;
    CommandId id;
    bool hasArgument;
};

static const CommandSpec COMMAND_TABLE[] = {
    {"F",        CommandId::Forward,   true},
    {"B",        CommandId::Backward,  true},
    {"L",        CommandId::Left90,    false},  // L = поворот 90°
    {"R",        CommandId::Right90,   false},  // R = поворот 90°
    {"paintON",  CommandId::PaintOn,   false},
    {"paintOFF", CommandId::PaintOff,  false},
    {"WAIT",     CommandId::Wait,      true},
};

static const size_t COMMAND_COUNT =
    sizeof(COMMAND_TABLE) / sizeof(COMMAND_TABLE[0]);

/* =========================================================
   Interfaces (сохранены преимущества старой архитектуры)
   ========================================================= */

class IMotionDriver {
public:
    virtual ~IMotionDriver() = default;
    virtual void forward() = 0;
    virtual void backward() = 0;
    virtual void rotateLeft() = 0;
    virtual void rotateRight() = 0;
    virtual void stop() = 0;
};

class IPainter {
public:
    virtual ~IPainter() = default;
    virtual void on() = 0;
    virtual void off() = 0;
};

class IClock {
public:
    virtual ~IClock() = default;
    virtual unsigned long now() const = 0;
};

class IFile {
public:
    virtual ~IFile() = default;
    virtual bool available() = 0;
    virtual String readLine() = 0;
};

class IFileSystem {
public:
    virtual ~IFileSystem() = default;
    virtual IFile* open(const String& name) = 0;
};

class IProgramSelector {
public:
    virtual ~IProgramSelector() = default;
    virtual String selectProgram() = 0;
};

class IRunControl {
public:
    virtual ~IRunControl() = default;
    virtual bool isRunEnabled() const = 0;
};

class IStartTrigger {
public:
    virtual ~IStartTrigger() = default;
    virtual bool startRequested() = 0; // фронт кнопки
};

/* =========================================================
   Robot = фасад над драйверами
   ========================================================= */

class Robot {
public:
    Robot(IMotionDriver& m, IPainter& p)
        : motion(m), painter(p) {}

    void forward()  { motion.forward(); }
    void backward() { motion.backward(); }
    void left90()   { motion.rotateLeft(); }
    void right90()  { motion.rotateRight(); }
    void stop()     { motion.stop(); }

    void paintOn()  { painter.on(); }
    void paintOff() { painter.off(); }

private:
    IMotionDriver& motion;
    IPainter& painter;
};

/* =========================================================
   Async command interface
   ========================================================= */

class ICommand {
public:
    virtual ~ICommand() = default;
    virtual void start(Robot& r, const IClock& clk) = 0;
    virtual void update(Robot& r, const IClock& clk) = 0;
    virtual bool finished() const = 0;
};

/* =========================================================
   Base: timed command
   ========================================================= */

class TimedCommand : public ICommand {
public:
    explicit TimedCommand(unsigned long d) : duration(d) {}

    void start(Robot&, const IClock& clk) override {
        startTime = clk.now();
        active = true;
    }

    void update(Robot& r, const IClock& clk) override {
        if (!active) return;
        if (clk.now() - startTime >= duration) {
            onFinish(r);
            active = false;
        }
    }

    bool finished() const override { return !active; }

protected:
    virtual void onFinish(Robot& r) = 0;

private:
    unsigned long duration;
    unsigned long startTime = 0;
    bool active = false;
};

/* =========================================================
   Concrete commands
   ========================================================= */

// Вперёд/назад: длительность в ms (для демонстрации)
// (в реальном роботе лучше заменить на энкодеры)
class CmdForward : public TimedCommand {
public:
    explicit CmdForward(unsigned long ms) : TimedCommand(ms) {}
    void start(Robot& r, const IClock& clk) override {
        r.forward();
        TimedCommand::start(r, clk);
    }
protected:
    void onFinish(Robot& r) override { r.stop(); }
};

class CmdBackward : public TimedCommand {
public:
    explicit CmdBackward(unsigned long ms) : TimedCommand(ms) {}
    void start(Robot& r, const IClock& clk) override {
        r.backward();
        TimedCommand::start(r, clk);
    }
protected:
    void onFinish(Robot& r) override { r.stop(); }
};

class CmdLeft90 : public TimedCommand {
public:
    CmdLeft90() : TimedCommand(ROTATE_90_MS) {}
    void start(Robot& r, const IClock& clk) override {
        r.left90();
        TimedCommand::start(r, clk);
    }
protected:
    void onFinish(Robot& r) override { r.stop(); }
private:
    static constexpr unsigned long ROTATE_90_MS = 420; // калибруется
};

class CmdRight90 : public TimedCommand {
public:
    CmdRight90() : TimedCommand(ROTATE_90_MS) {}
    void start(Robot& r, const IClock& clk) override {
        r.right90();
        TimedCommand::start(r, clk);
    }
protected:
    void onFinish(Robot& r) override { r.stop(); }
private:
    static constexpr unsigned long ROTATE_90_MS = 420; // калибруется
};

class CmdPaintOn : public ICommand {
public:
    void start(Robot& r, const IClock&) override { r.paintOn(); done = true; }
    void update(Robot&, const IClock&) override {}
    bool finished() const override { return done; }
private:
    bool done = false;
};

class CmdPaintOff : public ICommand {
public:
    void start(Robot& r, const IClock&) override { r.paintOff(); done = true; }
    void update(Robot&, const IClock&) override {}
    bool finished() const override { return done; }
private:
    bool done = false;
};

class CmdWait : public TimedCommand {
public:
    explicit CmdWait(unsigned long ms) : TimedCommand(ms) {}
    void start(Robot& r, const IClock& clk) override {
        r.stop();
        TimedCommand::start(r, clk);
    }
protected:
    void onFinish(Robot& r) override { r.stop(); }
};

/* =========================================================
   Parser & Factory
   ========================================================= */

class CommandFactory {
public:
    static ICommand* create(CommandId id, float value) {
        switch (id) {
            case CommandId::Forward:  return new CmdForward((unsigned long)value);
            case CommandId::Backward: return new CmdBackward((unsigned long)value);
            case CommandId::Left90:   return new CmdLeft90();
            case CommandId::Right90:  return new CmdRight90();
            case CommandId::PaintOn:  return new CmdPaintOn();
            case CommandId::PaintOff: return new CmdPaintOff();
            case CommandId::Wait:     return new CmdWait((unsigned long)value);
        }
        return nullptr;
    }
};

class CommandParser {
public:
    static ICommand* parse(String line) {
        line.trim();
        if (line.length() == 0) return nullptr;

        for (size_t i = 0; i < COMMAND_COUNT; ++i) {
            const CommandSpec& spec = COMMAND_TABLE[i];
            if (startsWithToken(line, spec.token)) {
                float value = 0;
                if (spec.hasArgument) value = extractArgument(line);
                return CommandFactory::create(spec.id, value);
            }
        }
        return nullptr;
    }

private:
    static bool startsWithToken(const String& line, const char* token) {
        if (!line.startsWith(token)) return false;
        int len = strlen(token);
        if (line.length() == len) return true;
        char c = line.charAt(len);
        return c == ' ' || c == '\t';
    }

    static float extractArgument(const String& line) {
        int p = line.indexOf(' ');
        if (p < 0) return 0;
        return line.substring(p + 1).toFloat();
    }
};

/* =========================================================
   Program runner (async + file)
   ========================================================= */

class ProgramRunner {
public:
    ProgramRunner(Robot& r, const IClock& c)
        : robot(r), clock(c) {}

    void start(IFile* f) {
        stop(); // очистить состояние
        file = f;
        running = (file != nullptr);
    }

    void stop() {
        if (current) {
            delete current;
            current = nullptr;
        }
        robot.stop();
        file = nullptr;
        running = false;
        finishedAll = false;
    }

    bool isRunning() const { return running; }
    bool isFinishedAll() const { return finishedAll; }

    void update() {
        if (!running) return;

        // если выполняем текущую команду — обновляем
        if (current) {
            current->update(robot, clock);
            if (current->finished()) {
                delete current;
                current = nullptr;
            }
            return;
        }

        // текущей команды нет — берём следующую строку
        if (!file || !file->available()) {
            running = false;
            finishedAll = true;
            robot.stop();
            return;
        }

        String line = file->readLine();
        Serial.println(line);

        current = CommandParser::parse(line);
        if (current) {
            current->start(robot, clock);
        }
        // если строка пустая/непонятная — просто пропускаем
    }

private:
    Robot& robot;
    const IClock& clock;

    IFile* file = nullptr;
    ICommand* current = nullptr;

    bool running = false;
    bool finishedAll = false;
};

/* =========================================================
   Memory file system (как раньше)
   ========================================================= */

class MemoryFile final : public IFile {
public:
    MemoryFile(const char* const* l, size_t c) : lines(l), count(c) {}
    bool available() override { return idx < count; }
    String readLine() override { return String(lines[idx++]); }
private:
    const char* const* lines;
    size_t count;
    size_t idx = 0;
};

class MemoryFileSystem final : public IFileSystem {
public:
    struct Entry {
        const char* name;
        const char* const* lines;
        size_t count;
    };

    MemoryFileSystem(const Entry* e, size_t c) : entries(e), n(c) {}

    IFile* open(const String& name) override {
        for (size_t i = 0; i < n; ++i) {
            if (name == entries[i].name) {
                delete active;
                active = new MemoryFile(entries[i].lines, entries[i].count);
                return active;
            }
        }
        return nullptr;
    }

    ~MemoryFileSystem() { delete active; }

private:
    const Entry* entries;
    size_t n;
    IFile* active = nullptr;
};

/* =========================================================
   Program selector (по умолчанию square.txt)
   ========================================================= */

class DefaultProgramSelector final : public IProgramSelector {
public:
    String selectProgram() override { return "square.txt"; }
};

/* =========================================================
   Run control (флаг “разрешено ли выполнять команды”)
   ========================================================= */

class RunControl final : public IRunControl {
public:
    void setEnabled(bool v) { enabled = v; }
    bool isRunEnabled() const override { return enabled; }
private:
    bool enabled = true;
};

/* =========================================================
   Start trigger (кнопка запуска по фронту)
   ========================================================= */

class ButtonStartTrigger final : public IStartTrigger {
public:
    explicit ButtonStartTrigger(uint8_t pin) : pin(pin) {}

    void begin() {
        pinMode(pin, INPUT_PULLUP);
    }

    bool startRequested() override {
        bool cur = !digitalRead(pin);     // нажата => true
        bool edge = cur && !last;         // фронт
        last = cur;
        return edge;
    }

private:
    uint8_t pin;
    bool last = false;
};

/* =========================================================
   Hardware implementations (пример под H-bridge)
   ========================================================= */

class ArduinoClock final : public IClock {
public:
    unsigned long now() const override { return millis(); }
};

class MotorDriver final : public IMotionDriver {
public:
    MotorDriver(uint8_t lf, uint8_t lb, uint8_t rf, uint8_t rb)
        : LF(lf), LB(lb), RF(rf), RB(rb) {}

    void begin() {
        pinMode(LF, OUTPUT);
        pinMode(LB, OUTPUT);
        pinMode(RF, OUTPUT);
        pinMode(RB, OUTPUT);
        stop();
    }

    void forward() override {
        digitalWrite(LF, HIGH); digitalWrite(LB, LOW);
        digitalWrite(RF, HIGH); digitalWrite(RB, LOW);
    }

    void backward() override {
        digitalWrite(LF, LOW);  digitalWrite(LB, HIGH);
        digitalWrite(RF, LOW);  digitalWrite(RB, HIGH);
    }

    void rotateLeft() override {
        digitalWrite(LF, LOW);  digitalWrite(LB, HIGH);
        digitalWrite(RF, HIGH); digitalWrite(RB, LOW);
    }

    void rotateRight() override {
        digitalWrite(LF, HIGH); digitalWrite(LB, LOW);
        digitalWrite(RF, LOW);  digitalWrite(RB, HIGH);
    }

    void stop() override {
        digitalWrite(LF, LOW); digitalWrite(LB, LOW);
        digitalWrite(RF, LOW); digitalWrite(RB, LOW);
    }

private:
    uint8_t LF, LB, RF, RB;
};

class DummyPainter final : public IPainter {
public:
    void on() override  { Serial.println(F("[PAINT] ON")); }
    void off() override { Serial.println(F("[PAINT] OFF")); }
};

/* =========================================================
   Demo programs (из “файлов”)
   Формат:
     F <ms>
     B <ms>
     L          (90°)
     R          (90°)
     WAIT <ms>
     paintON / paintOFF
   ========================================================= */

static const char* const SQUARE[] = {
    "paintON",
    "F 700",
    "R",
    "F 700",
    "R",
    "F 700",
    "R",
    "F 700",
    "R",
    "paintOFF"
};

static const char* const TRIANGLE[] = {
    "paintON",
    "F 700",
    "R",
    "R",
    "F 700",
    "R",
    "R",
    "F 700",
    "paintOFF"
};

static const MemoryFileSystem::Entry FS_ENTRIES[] = {
    {"square.txt",   SQUARE,   sizeof(SQUARE) / sizeof(SQUARE[0])},
    {"triangle.txt", TRIANGLE, sizeof(TRIANGLE) / sizeof(TRIANGLE[0])},
};

/* =========================================================
   Program controller (склеивает всё: selector + FS + trigger)
   ========================================================= */

class ProgramController {
public:
    ProgramController(IProgramSelector& sel,
                      IFileSystem& fs,
                      ProgramRunner& runner,
                      IStartTrigger& trigger,
                      const IRunControl& runCtl)
        : selector(sel), filesystem(fs), runner(runner),
          trigger(trigger), runControl(runCtl) {}

    void update() {
        if (trigger.startRequested()) {
            if (!runControl.isRunEnabled()) {
                Serial.println(F("[RUN] blocked by runControl"));
            } else if (runner.isRunning()) {
                Serial.println(F("[RUN] already running"));
            } else {
                String name = selector.selectProgram();
                IFile* f = filesystem.open(name);
                if (!f) {
                    Serial.print(F("[RUN] file not found: "));
                    Serial.println(name);
                } else {
                    Serial.print(F("[RUN] start: "));
                    Serial.println(name);
                    runner.start(f);
                }
            }
        }

        // 2) обновляем выполнение программы
        if (runControl.isRunEnabled()) {
            runner.update();
            if (runner.isFinishedAll()) {
                Serial.println(F("[RUN] finished"));
            }
        }
    }

private:
    IProgramSelector& selector;
    IFileSystem& filesystem;
    ProgramRunner& runner;
    IStartTrigger& trigger;
    const IRunControl& runControl;
};

/* =========================================================
   Globals / Wiring
   ========================================================= */

// Пины моторов (пример)
MotorDriver motors(/*LF*/5, /*LB*/6, /*RF*/9, /*RB*/10);
DummyPainter painter;
ArduinoClock clockImpl;

Robot robot(motors, painter);
ProgramRunner runner(robot, clockImpl);

MemoryFileSystem fs(FS_ENTRIES, sizeof(FS_ENTRIES) / sizeof(FS_ENTRIES[0]));
DefaultProgramSelector selector;

RunControl runControl;

// Кнопка запуска программы (по нажатию)
ButtonStartTrigger startButton(/*pin*/2);

ProgramController controller(selector, fs, runner, startButton, runControl);

/* =========================================================
   Arduino entry points
   ========================================================= */

void setup() {
    Serial.begin(115200);
    while (!Serial) {}

    motors.begin();
    startButton.begin();

    Serial.println(F("Ready. Press button on pin 2 to start program."));
}

void loop() {
    controller.update();
}
