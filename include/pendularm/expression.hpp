#ifndef PENDULARM_EXPRESSION_HPP
#define PENDULARM_EXPRESSION_HPP

#include <string>
#include <vector>
#include <memory>
#include <stdexcept>
#include <unordered_set>

namespace pendularm {

// *CHANGES*: Separated declarations from definitions into src/expression.cpp
// per updated IMPLEMENT.md guideline on source/header separation.

// Custom exception for math expression parsing and lexical analysis errors
class ParseError : public std::runtime_error {
public:
    explicit ParseError(const std::string& msg);
};

// Abstract Syntax Tree (AST) base node
class ExprNode {
public:
    virtual ~ExprNode() = default;
    virtual double evaluate(double t) const = 0;
};

using ExprPtr = std::unique_ptr<ExprNode>;

// Numeric constant node
class NumberNode : public ExprNode {
private:
    double val;
public:
    explicit NumberNode(double v);
    double evaluate(double t) const override;
};

// Variable node representing 't'
class VariableNode : public ExprNode {
public:
    VariableNode() = default;
    double evaluate(double t) const override;
};

// Unary operator node (prefix '-' or '+')
class UnaryOpNode : public ExprNode {
private:
    char op;
    ExprPtr child;
public:
    UnaryOpNode(char o, ExprPtr c);
    double evaluate(double t) const override;
};

// Binary operator node (+, -, *, /, ^)
class BinaryOpNode : public ExprNode {
private:
    char op;
    ExprPtr left;
    ExprPtr right;
public:
    BinaryOpNode(char o, ExprPtr l, ExprPtr r);
    double evaluate(double t) const override;
};

// Single-argument function call node (sin, cos, tan, exp, sqrt, ln, abs)
class FunctionCallNode : public ExprNode {
private:
    std::string name;
    ExprPtr arg;
public:
    FunctionCallNode(std::string n, ExprPtr a);
    double evaluate(double t) const override;
};

// Token types for lexical analysis
enum class TokenType {
    Number,
    Variable,
    Function,
    Plus,
    Minus,
    Multiply,
    Divide,
    Power,
    LeftParen,
    RightParen,
    Eof
};

struct Token {
    TokenType type{TokenType::Eof};
    std::string text;
    double number_value{0.0};
    size_t pos{0};
};

class Lexer {
private:
    std::string input;
    size_t cursor{0};

    static const std::unordered_set<std::string>& valid_functions();

public:
    explicit Lexer(std::string str);
    std::vector<Token> tokenize();
};

/*
Recursive descent parser respecting operator precedence:
Lowest to highest:
1. + - (left-associative)
2. * / (left-associative)
3. ^   (right-associative)
4. unary - (higher precedence than ^ per specification)
5. atoms: number, variable t, function call, (expression)
*/
class Parser {
private:
    std::vector<Token> tokens;
    size_t current{0};

    const Token& next();
    bool match(TokenType type);

public:
    explicit Parser(std::vector<Token> toks);

    const Token& peek() const;
    bool is_at_end() const;

    ExprPtr parse_additive();
    ExprPtr parse_multiplicative();
    ExprPtr parse_power();
    ExprPtr parse_unary();
    ExprPtr parse_atom();
};

// Top-level expression parser entry point
ExprPtr parse_expression(const std::string& expr_str);

} // namespace pendularm

#endif // PENDULARM_EXPRESSION_HPP
