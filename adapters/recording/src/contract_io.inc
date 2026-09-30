// Shared by the loaders of this library's contracts, the recording and the reference-optimal lap: a JSON reader and
// a CSV table reader. Not a header: it is included inside each loader's anonymous namespace, after that loader's own
// fail(), which every violation found here is reported through, and after the standard headers it uses (<algorithm>,
// <cctype>, <cmath>, <filesystem>, <fstream>, <sstream>, <string>, <string_view>, <vector>).

// ---------------------------------------------------------------- JSON

struct Json {
    enum class Type { Null, Bool, Number, String, Array, Object };
    Type type{Type::Null};
    bool boolean{};
    double number{};
    std::string string;
    std::vector<Json> array;
    std::vector<std::string> keys;    // object members, in file order
    std::vector<Json> values;
    const Json* find(const std::string& key) const {
        if (type != Type::Object) return nullptr;
        for (std::size_t i = 0; i < keys.size(); ++i) if (keys[i] == key) return &values[i];
        return nullptr;
    }
};

double parse_number(const std::string& text, const std::string& what) {
    std::size_t end{};
    double value{};
    try { value = std::stod(text, &end); }
    catch (const std::exception&) { fail(what+" is not a number: '"+text+"'"); }
    if (end != text.size() || !std::isfinite(value)) fail(what+" is not a finite number: '"+text+"'");
    return value;
}

class JsonParser {
public:
    explicit JsonParser(std::string_view text) : text_(text) {}
    Json document() {
        Json value = parse_value();
        skip_space();
        if (position_ != text_.size()) fail("JSON has trailing characters");
        return value;
    }
private:
    void skip_space() {
        while (position_ < text_.size() && std::isspace(static_cast<unsigned char>(text_[position_]))) ++position_;
    }
    char peek() {
        skip_space();
        if (position_ >= text_.size()) fail("JSON ends unexpectedly");
        return text_[position_];
    }
    void expect(char c) {
        if (peek() != c) fail(std::string("JSON expected '")+c+"'");
        ++position_;
    }
    bool literal(std::string_view word) {
        if (text_.compare(position_, word.size(), word) != 0) return false;
        position_ += word.size();
        return true;
    }
    Json parse_value() {
        const char c = peek();
        Json value;
        if (c == '{') {
            ++position_;
            value.type = Json::Type::Object;
            if (peek() == '}') { ++position_; return value; }
            for (;;) {
                std::string key = parse_string();
                expect(':');
                Json item = parse_value();
                if (value.find(key)) fail("JSON duplicate key "+key);
                value.keys.push_back(std::move(key));
                value.values.push_back(std::move(item));
                const char next = peek();
                ++position_;
                if (next == '}') break;
                if (next != ',') fail("JSON expected ',' or '}'");
            }
            return value;
        }
        if (c == '[') {
            ++position_;
            value.type = Json::Type::Array;
            if (peek() == ']') { ++position_; return value; }
            for (;;) {
                value.array.push_back(parse_value());
                const char next = peek();
                ++position_;
                if (next == ']') break;
                if (next != ',') fail("JSON expected ',' or ']'");
            }
            return value;
        }
        if (c == '"') { value.type = Json::Type::String; value.string = parse_string(); return value; }
        if (literal("true")) { value.type = Json::Type::Bool; value.boolean = true; return value; }
        if (literal("false")) { value.type = Json::Type::Bool; value.boolean = false; return value; }
        if (literal("null")) return value;
        const std::size_t start = position_;
        while (position_ < text_.size()) {
            const char d = text_[position_];
            if (std::isdigit(static_cast<unsigned char>(d)) || d=='-' || d=='+' || d=='.' || d=='e' || d=='E') ++position_;
            else break;
        }
        if (start == position_) fail("JSON unexpected character");
        value.type = Json::Type::Number;
        value.number = parse_number(std::string(text_.substr(start, position_-start)), "JSON value");
        return value;
    }
    std::string parse_string() {
        expect('"');
        std::string out;
        for (;;) {
            if (position_ >= text_.size()) fail("JSON unterminated string");
            const char c = text_[position_++];
            if (c == '"') break;
            if (c != '\\') { out += c; continue; }
            if (position_ >= text_.size()) fail("JSON unterminated escape");
            const char e = text_[position_++];
            switch (e) {
            case '"': out += '"'; break;
            case '\\': out += '\\'; break;
            case '/': out += '/'; break;
            case 'n': out += '\n'; break;
            case 'r': out += '\r'; break;
            case 't': out += '\t'; break;
            case 'b': out += '\b'; break;
            case 'f': out += '\f'; break;
            case 'u': {
                if (position_+4 > text_.size()) fail("JSON truncated \\u escape");
                const std::string hex(text_.substr(position_, 4));
                std::size_t end{};
                unsigned long code{};
                try { code = std::stoul(hex, &end, 16); } catch (const std::exception&) { end = 0; }
                if (end != 4) fail("JSON invalid \\u escape");
                position_ += 4;
                // The writer only emits control characters this way; encode BMP code points.
                if (code < 0x80) out += static_cast<char>(code);
                else if (code < 0x800) { out += static_cast<char>(0xC0|(code>>6)); out += static_cast<char>(0x80|(code&0x3F)); }
                else { out += static_cast<char>(0xE0|(code>>12)); out += static_cast<char>(0x80|((code>>6)&0x3F)); out += static_cast<char>(0x80|(code&0x3F)); }
                break;
            }
            default: fail("JSON invalid escape");
            }
        }
        return out;
    }
    std::string_view text_;
    std::size_t position_{};
};

Json read_json(const std::filesystem::path& path) {
    std::ifstream file(path, std::ios::binary);
    if (!file) fail("missing file "+path.filename().string());
    std::stringstream buffer;
    buffer << file.rdbuf();
    const std::string text = buffer.str();
    try { return JsonParser(text).document(); }
    catch (const std::runtime_error& error) { fail(path.filename().string()+": "+error.what()); }
}

const Json& member(const Json& object, const std::string& key, const std::string& where) {
    const Json* found = object.find(key);
    if (!found) fail(where+" lacks \""+key+"\"");
    return *found;
}
double number_member(const Json& object, const std::string& key, const std::string& where) {
    const Json& value = member(object, key, where);
    if (value.type != Json::Type::Number) fail(where+"."+key+" must be a number");
    return value.number;
}
int integer_member(const Json& object, const std::string& key, const std::string& where) {
    const double value = number_member(object, key, where);
    if (std::floor(value) != value || std::abs(value) > 1e9) fail(where+"."+key+" must be an integer");
    return static_cast<int>(value);
}
std::string string_member(const Json& object, const std::string& key, const std::string& where) {
    const Json& value = member(object, key, where);
    if (value.type != Json::Type::String) fail(where+"."+key+" must be a string");
    return value.string;
}
bool bool_member(const Json& object, const std::string& key, const std::string& where) {
    const Json& value = member(object, key, where);
    if (value.type != Json::Type::Bool) fail(where+"."+key+" must be true or false");
    return value.boolean;
}
const Json& object_member(const Json& object, const std::string& key, const std::string& where) {
    const Json& value = member(object, key, where);
    if (value.type != Json::Type::Object) fail(where+"."+key+" must be an object");
    return value;
}
const Json& array_member(const Json& object, const std::string& key, const std::string& where) {
    const Json& value = member(object, key, where);
    if (value.type != Json::Type::Array) fail(where+"."+key+" must be an array");
    return value;
}

// ---------------------------------------------------------------- CSV

struct Table {
    std::vector<std::string> columns;
    std::vector<std::vector<std::string>> rows;
    std::string file;
    std::size_t column(const std::string& name) const {
        const auto it = std::find(columns.begin(), columns.end(), name);
        if (it == columns.end()) fail(file+" lacks column "+name);
        return static_cast<std::size_t>(it-columns.begin());
    }
};

std::vector<std::string> split(const std::string& line) {
    std::vector<std::string> fields;
    std::string current;
    for (const char c : line) {
        if (c == ',') { fields.push_back(current); current.clear(); }
        else current += c;
    }
    fields.push_back(current);
    return fields;
}

Table read_csv(const std::filesystem::path& path, const std::string& expected_header) {
    std::ifstream file(path);
    if (!file) fail("missing file "+path.filename().string());
    Table table;
    table.file = path.filename().string();
    std::string line;
    if (!std::getline(file, line)) fail(table.file+" is empty");
    if (!line.empty() && line.back() == '\r') line.pop_back();
    if (line != expected_header) fail(table.file+" header does not match the recording contract");
    table.columns = split(line);
    std::size_t number = 1;
    while (std::getline(file, line)) {
        ++number;
        if (!line.empty() && line.back() == '\r') line.pop_back();
        if (line.empty()) fail(table.file+" line "+std::to_string(number)+" is blank");
        auto fields = split(line);
        if (fields.size() != table.columns.size()) fail(table.file+" line "+std::to_string(number)+" has the wrong number of fields");
        table.rows.push_back(std::move(fields));
    }
    return table;
}

class RowReader {
public:
    RowReader(const Table& table, std::size_t row) : table_(table), row_(row) {}
    std::string where(const std::string& column) const { return table_.file+" line "+std::to_string(row_+2)+" "+column; }
    const std::string& text(const std::string& column) const { return table_.rows[row_][table_.column(column)]; }
    double number(const std::string& column) const { return parse_number(text(column), where(column)); }
    std::uint64_t index(const std::string& column) const {
        const std::string& value = text(column);
        if (value.empty() || value.find_first_not_of("0123456789") != std::string::npos || value.size() > 18)
            fail(where(column)+" must be a nonnegative integer");
        return std::stoull(value);
    }
    bool flag(const std::string& column) const {
        const std::string& value = text(column);
        if (value != "0" && value != "1") fail(where(column)+" must be 0 or 1");
        return value == "1";
    }
private:
    const Table& table_;
    std::size_t row_;
};
