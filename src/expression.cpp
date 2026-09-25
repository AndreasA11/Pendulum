#include "pendularm/expression.hpp"
#include <cmath>
#include <cctype>
#include <sstream>

namespace pendularm {

// *CHANGES*: Extracted expression parser and AST implementations from header to source file
// per updated IMPLEMENT.md guideline.

ParseError::ParseError(const std::string& msg) : std::runtime_error(msg) {}

NumberNode::NumberNode(double v) : val(v) {}
double NumberNode::evaluate(double /*t*/) const {
    return val;
}

double VariableNode::evaluate(double t) const {
    return t;
}

UnaryOpNode::UnaryOpNode(char o, ExprPtr c) : op(o), child(std::move(c)) {}
double UnaryOpNode::evaluate(double t) const {
    double v = child->evaluate(t);
    if (op == '-') return -v;
    return v;
}

BinaryOpNode::BinaryOpNode(char o, ExprPtr l, ExprPtr r)
    : op(o), left(std::move(l)), right(std::move(r)) {}

double BinaryOpNode::evaluate(double t) const {
    double l_val = left->evaluate(t);
    double r_val = right->evaluate(t);
    switch (op) {
        case '+': return l_val + r_val;
        case '-': return l_val - r_val;
        case '*': return l_val * r_val;
        case '/': return l_val / r_val;
        case '^': return std::pow(l_val, r_val);
        default: return 0.0;
    }
}

FunctionCallNode::FunctionCallNode(std::string n, ExprPtr a)
    : name(std::move(n)), arg(std::move(a)) {}

double FunctionCallNode::evaluate(double t) const {
    double a_val = arg->evaluate(t);
    if (name == "sin") return std::sin(a_val);
    if (name == "cos") return std::cos(a_val);
    if (name == "tan") return std::tan(a_val);
    if (name == "exp") return std::exp(a_val);
    if (name == "sqrt") return std::sqrt(a_val);
    if (name == "ln") return std::log(a_val);
    if (name == "abs") return std::fabs(a_val);
    return 0.0;
}

const std::unordered_set<std::string>& Lexer::valid_functions() {
    static const std::unordered_set<std::string> funcs = {
        "sin", "cos", "tan", "exp", "sqrt", "ln", "abs"
    };
    return funcs;
}

Lexer::Lexer(std::string str) : input(std::move(str)), cursor(0) {}

std::vector<Token> Lexer::tokenize() {
    std::vector<Token> tokens;
    while (cursor < input.size()) {
        char c = input[cursor];

        // Skip whitespace
        if (std::isspace(static_cast<unsigned char>(c))) {
            ++cursor;
            continue;
        }

        // Identifiers: either variable 't' or named functions
        if (std::isalpha(static_cast<unsigned char>(c))) {
            size_t start = cursor;
            while (cursor < input.size() && std::isalpha(static_cast<unsigned char>(input[cursor]))) {
                ++cursor;
            }
            std::string ident = input.substr(start, cursor - start);
            if (ident == "t") {
                tokens.push_back({TokenType::Variable, ident, 0.0, start});
            } else if (valid_functions().find(ident) != valid_functions().end()) {
                tokens.push_back({TokenType::Function, ident, 0.0, start});
            } else {
                throw ParseError("Unknown identifier '" + ident + "' at position " + std::to_string(start));
            }
            continue;
        }

        // Numbers: integer or floating point, optional exponent, optional leading decimal dot
        if (std::isdigit(static_cast<unsigned char>(c)) ||
            (c == '.' && cursor + 1 < input.size() && std::isdigit(static_cast<unsigned char>(input[cursor + 1])))) {
            size_t start = cursor;
            bool has_dot = false;
            if (input[cursor] == '.') {
                has_dot = true;
                ++cursor;
            }
            while (cursor < input.size() && std::isdigit(static_cast<unsigned char>(input[cursor]))) {
                ++cursor;
            }
            if (!has_dot && cursor < input.size() && input[cursor] == '.') {
                has_dot = true;
                ++cursor;
                while (cursor < input.size() && std::isdigit(static_cast<unsigned char>(input[cursor]))) {
                    ++cursor;
                }
            }
            // Scientific notation: e or E
            if (cursor < input.size() && (input[cursor] == 'e' || input[cursor] == 'E')) {
                size_t e_pos = cursor;
                ++cursor;
                if (cursor < input.size() && (input[cursor] == '+' || input[cursor] == '-')) {
                    ++cursor;
                }
                if (cursor >= input.size() || !std::isdigit(static_cast<unsigned char>(input[cursor]))) {
                    throw ParseError("Malformed exponent in number literal at position " + std::to_string(e_pos));
                }
                while (cursor < input.size() && std::isdigit(static_cast<unsigned char>(input[cursor]))) {
                    ++cursor;
                }
            }
            // Check for multiple dots
            if (cursor < input.size() && input[cursor] == '.') {
                throw ParseError("Malformed number with multiple decimal points at position " + std::to_string(cursor));
            }
            // Check if immediately adjacent to an identifier without operator (e.g. "2t")
            if (cursor < input.size() && std::isalpha(static_cast<unsigned char>(input[cursor]))) {
                throw ParseError("Missing operator between number and identifier at position " + std::to_string(cursor));
            }

            std::string num_str = input.substr(start, cursor - start);
            double val = std::stod(num_str);
            tokens.push_back({TokenType::Number, num_str, val, start});
            continue;
        }

        // Operators and punctuation
        size_t start = cursor++;
        switch (c) {
            case '+': tokens.push_back({TokenType::Plus, "+", 0.0, start}); break;
            case '-': tokens.push_back({TokenType::Minus, "-", 0.0, start}); break;
            case '*': tokens.push_back({TokenType::Multiply, "*", 0.0, start}); break;
            case '/': tokens.push_back({TokenType::Divide, "/", 0.0, start}); break;
            case '^': tokens.push_back({TokenType::Power, "^", 0.0, start}); break;
            case '(': tokens.push_back({TokenType::LeftParen, "(", 0.0, start}); break;
            case ')': tokens.push_back({TokenType::RightParen, ")", 0.0, start}); break;
            default:
                throw ParseError(std::string("Unsupported character '") + c + "' at position " + std::to_string(start));
        }
    }

    tokens.push_back({TokenType::Eof, "", 0.0, cursor});
    return tokens;
}

Parser::Parser(std::vector<Token> toks) : tokens(std::move(toks)), current(0) {}

const Token& Parser::peek() const {
    if (current < tokens.size()) return tokens[current];
    return tokens.back();
}

const Token& Parser::next() {
    const Token& tok = peek();
    if (current < tokens.size() && tokens[current].type != TokenType::Eof) {
        ++current;
    }
    return tok;
}

bool Parser::match(TokenType type) {
    if (peek().type == type) {
        next();
        return true;
    }
    return false;
}

bool Parser::is_at_end() const {
    return peek().type == TokenType::Eof;
}

ExprPtr Parser::parse_additive() {
    auto left = parse_multiplicative();
    while (peek().type == TokenType::Plus || peek().type == TokenType::Minus) {
        char op = (peek().type == TokenType::Plus) ? '+' : '-';
        next();
        auto right = parse_multiplicative();
        left = std::make_unique<BinaryOpNode>(op, std::move(left), std::move(right));
    }
    return left;
}

ExprPtr Parser::parse_multiplicative() {
    auto left = parse_power();
    while (peek().type == TokenType::Multiply || peek().type == TokenType::Divide) {
        char op = (peek().type == TokenType::Multiply) ? '*' : '/';
        next();
        auto right = parse_power();
        left = std::make_unique<BinaryOpNode>(op, std::move(left), std::move(right));
    }
    return left;
}

ExprPtr Parser::parse_power() {
    auto left = parse_unary();
    if (peek().type == TokenType::Power) {
        next();
        auto right = parse_power(); // right-associative recursion
        return std::make_unique<BinaryOpNode>('^', std::move(left), std::move(right));
    }
    return left;
}

ExprPtr Parser::parse_unary() {
    if (peek().type == TokenType::Minus) {
        next();
        auto child = parse_unary();
        return std::make_unique<UnaryOpNode>('-', std::move(child));
    }
    if (peek().type == TokenType::Plus) {
        next();
        return parse_unary();
    }
    return parse_atom();
}

ExprPtr Parser::parse_atom() {
    const Token& tok = peek();
    if (tok.type == TokenType::Number) {
        next();
        return std::make_unique<NumberNode>(tok.number_value);
    }
    if (tok.type == TokenType::Variable) {
        next();
        return std::make_unique<VariableNode>();
    }
    if (tok.type == TokenType::Function) {
        std::string fn_name = tok.text;
        size_t fn_pos = tok.pos;
        next();
        if (peek().type != TokenType::LeftParen) {
            throw ParseError("Expected '(' after function '" + fn_name + "' at position " + std::to_string(fn_pos));
        }
        next(); // consume '('
        auto arg = parse_additive();
        if (peek().type != TokenType::RightParen) {
            throw ParseError("Expected ')' closing argument for function '" + fn_name + "' at position " + std::to_string(peek().pos));
        }
        next(); // consume ')'
        return std::make_unique<FunctionCallNode>(fn_name, std::move(arg));
    }
    if (tok.type == TokenType::LeftParen) {
        size_t paren_pos = tok.pos;
        next();
        auto expr = parse_additive();
        if (peek().type != TokenType::RightParen) {
            throw ParseError("Unclosed parenthesis opened at position " + std::to_string(paren_pos));
        }
        next(); // consume ')'
        return expr;
    }
    if (tok.type == TokenType::RightParen) {
        throw ParseError("Unexpected ')' with no matching '(' at position " + std::to_string(tok.pos));
    }
    if (tok.type == TokenType::Eof) {
        throw ParseError("Unexpected end of expression");
    }
    throw ParseError("Unexpected token '" + tok.text + "' at position " + std::to_string(tok.pos));
}

ExprPtr parse_expression(const std::string& expr_str) {
    Lexer lexer(expr_str);
    std::vector<Token> tokens = lexer.tokenize();

    if (tokens.empty() || (tokens.size() == 1 && tokens[0].type == TokenType::Eof)) {
        throw ParseError("Expression string is empty");
    }

    Parser parser(std::move(tokens));
    ExprPtr ast = parser.parse_additive();

    if (!parser.is_at_end()) {
        throw ParseError("Unexpected trailing tokens starting with '" + parser.peek().text + "'");
    }

    return ast;
}

} // namespace pendularm
