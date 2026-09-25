#include <iostream>
#include <cassert>
#include <string>
#include <vector>
#include <thread>
#include <chrono>
#include <sys/socket.h>
#include <netinet/in.h>
#include <arpa/inet.h>
#include <unistd.h>
#include "middleware/json.hpp"
#include "middleware/middleware.hpp"
#include "middleware/rosbridge_server.hpp"

using namespace middleware;

/*
1. Connect a TCP socket to the specified host and port.
2. Return the connected socket file descriptor, or -1 on failure.
*/
int connect_tcp(const std::string& host, int port) {
    // Associated with Step 1: Create TCP socket
    int sock = socket(AF_INET, SOCK_STREAM, 0);
    if (sock < 0) return -1;

    struct sockaddr_in addr{};
    addr.sin_family = AF_INET;
    addr.sin_port = htons(static_cast<uint16_t>(port));
    inet_pton(AF_INET, host.c_str(), &addr.sin_addr);

    // Associated with Step 2: Connect and return descriptor
    if (connect(sock, reinterpret_cast<struct sockaddr*>(&addr), sizeof(addr)) < 0) {
        close(sock);
        return -1;
    }
    return sock;
}

/*
1. Send a string message followed by a newline '\n' to a TCP socket.
2. Return true if all bytes were sent, false otherwise.
*/
bool send_tcp_line(int sock, const std::string& msg) {
    // Associated with Step 1: Append newline delimiter
    std::string payload = msg + "\n";
    size_t total = payload.size();
    size_t sent = 0;
    const char* buf = payload.data();

    // Associated with Step 2: Loop until all bytes are sent
    while (sent < total) {
        ssize_t n = send(sock, buf + sent, total - sent, 0);
        if (n <= 0) return false;
        sent += static_cast<size_t>(n);
    }
    return true;
}

/*
1. Read bytes from a TCP socket with a timeout until a complete newline '\n' is received.
2. Return the extracted line without trailing '\r' or '\n', or empty string on timeout/error.
*/
std::string read_tcp_line(int sock, int timeout_ms = 2000) {
    std::string result;
    auto start = std::chrono::steady_clock::now();

    // Set socket receive timeout
    struct timeval tv{};
    tv.tv_sec = timeout_ms / 1000;
    tv.tv_usec = (timeout_ms % 1000) * 1000;
    setsockopt(sock, SOL_SOCKET, SO_RCVTIMEO, &tv, sizeof(tv));

    // Associated with Step 1: Read byte-by-byte until newline
    char c = 0;
    while (true) {
        ssize_t n = recv(sock, &c, 1, 0);
        if (n <= 0) break;
        if (c == '\n') break;
        if (c != '\r') result += c;

        auto now = std::chrono::steady_clock::now();
        if (std::chrono::duration_cast<std::chrono::milliseconds>(now - start).count() > timeout_ms) {
            break;
        }
    }
    // Associated with Step 2: Return extracted line
    return result;
}

/*
1. Test in-process Middleware topic publishing and subscription.
2. Verify topic isolation so messages published to one topic do not leak to another.
3. Verify multiple subscribers on the same topic receive publications.
4. Verify unsubscription cleanly removes subscriber.
*/
void test_in_process_topics() {
    std::cout << "[TEST] In-process Middleware Topics..." << std::endl;
    Middleware mw;

    std::vector<std::string> topic1_msgs;
    std::vector<std::string> topic2_msgs;
    std::vector<std::string> topic1_sub2_msgs;

    // Associated with Step 1 & 2: Subscribe to topic1 and topic2
    SubscriptionId s1 = mw.subscribe("/test/topic1", [&](const JsonValue& msg) {
        topic1_msgs.push_back(msg["data"].as_string());
    });

    SubscriptionId s2 = mw.subscribe("/test/topic2", [&](const JsonValue& msg) {
        topic2_msgs.push_back(msg["data"].as_string());
    });

    // Associated with Step 3: Second subscriber to topic1
    SubscriptionId s3 = mw.subscribe("/test/topic1", [&](const JsonValue& msg) {
        topic1_sub2_msgs.push_back(msg["data"].as_string());
    });

    // Publish to topic1
    JsonValue msg1 = JsonValue::make_object();
    msg1["data"] = "message_A";
    mw.publish("/test/topic1", msg1);

    // Verify topic1 received and topic2 did NOT (isolation)
    assert(topic1_msgs.size() == 1 && topic1_msgs[0] == "message_A");
    assert(topic1_sub2_msgs.size() == 1 && topic1_sub2_msgs[0] == "message_A");
    assert(topic2_msgs.empty());

    // Publish to topic2
    JsonValue msg2 = JsonValue::make_object();
    msg2["data"] = "message_B";
    mw.publish("/test/topic2", msg2);

    assert(topic2_msgs.size() == 1 && topic2_msgs[0] == "message_B");
    assert(topic1_msgs.size() == 1);

    // Associated with Step 4: Unsubscribe s3 and publish again
    bool unsub_res = mw.unsubscribe("/test/topic1", s3);
    assert(unsub_res);

    JsonValue msg3 = JsonValue::make_object();
    msg3["data"] = "message_C";
    mw.publish("/test/topic1", msg3);

    assert(topic1_msgs.size() == 2 && topic1_msgs[1] == "message_C");
    assert(topic1_sub2_msgs.size() == 1); // s3 did not receive message_C

    mw.unsubscribe("/test/topic1", s1);
    mw.unsubscribe("/test/topic2", s2);
    std::cout << "  -> PASSED" << std::endl;
}

/*
1. Test in-process Middleware service registration and invocation.
2. Verify service execution returns correct results.
3. Verify calling nonexistent service returns failure.
4. Verify unadvertising service prevents future calls.
*/
void test_in_process_services() {
    std::cout << "[TEST] In-process Middleware Services..." << std::endl;
    Middleware mw;

    // Associated with Step 1 & 2: Register adder service
    bool adv = mw.advertise_service("/add_two_ints", [](const JsonValue& args) -> std::pair<bool, JsonValue> {
        double a = args["a"].as_number();
        double b = args["b"].as_number();
        JsonValue res = JsonValue::make_object();
        res["sum"] = a + b;
        return {true, res};
    });
    assert(adv);

    // Duplicate registration must fail
    assert(!mw.advertise_service("/add_two_ints", [](const JsonValue&) {
        return std::make_pair(false, JsonValue());
    }));

    // Call service
    JsonValue req = JsonValue::make_object();
    req["a"] = 15;
    req["b"] = 27;
    auto result = mw.call_service("/add_two_ints", req);
    assert(result.first == true);
    assert(result.second["sum"].as_int() == 42);

    // Associated with Step 3: Call nonexistent service
    auto not_found = mw.call_service("/nonexistent_service", req);
    assert(not_found.first == false);

    // Associated with Step 4: Unadvertise service
    bool unadv = mw.unadvertise_service("/add_two_ints");
    assert(unadv);
    auto unadv_call = mw.call_service("/add_two_ints", req);
    assert(unadv_call.first == false);

    std::cout << "  -> PASSED" << std::endl;
}

/*
1. Test external TCP/JSON gateway connection at 127.0.0.1:9095.
2. Verify client receives informational status upon subscribing (ROSBRIDGE_PROTOCOL.md line 35).
3. Verify client can subscribe and receive published messages.
4. Verify topic isolation over TCP.
*/
void test_tcp_gateway_pubsub() {
    std::cout << "[TEST] TCP Gateway Pub/Sub & Status (127.0.0.1:9095)..." << std::endl;
    Middleware mw;
    int test_port = 9095;
    RosbridgeServer server(mw, "127.0.0.1", test_port);
    assert(server.start());
    std::this_thread::sleep_for(std::chrono::milliseconds(50));

    // Connect Client 1 (Subscriber to /alpha)
    int client1 = connect_tcp("127.0.0.1", test_port);
    assert(client1 >= 0 && "Client 1 connection must succeed on 127.0.0.1:9095");

    // Connect Client 2 (Subscriber to /beta)
    int client2 = connect_tcp("127.0.0.1", test_port);
    assert(client2 >= 0 && "Client 2 connection must succeed on 127.0.0.1:9095");

    // Connect Client 3 (Publisher)
    int client3 = connect_tcp("127.0.0.1", test_port);
    assert(client3 >= 0 && "Client 3 connection must succeed on 127.0.0.1:9095");

    // Client 1 subscribes to /alpha with id "sub-alpha-1"
    assert(send_tcp_line(client1, "{\"op\":\"subscribe\",\"topic\":\"/alpha\",\"id\":\"sub-alpha-1\"}"));

    // Associated with Step 2: Client 1 must receive informational status confirming subscription
    std::string stat1 = read_tcp_line(client1);
    assert(!stat1.empty());
    JsonValue parsed_stat1 = JsonValue::parse(stat1);
    assert(parsed_stat1["op"].as_string() == "status");
    assert(parsed_stat1["level"].as_string() == "info");
    assert(parsed_stat1["msg"].as_string() == "subscribed to /alpha");
    assert(parsed_stat1["id"].as_string() == "sub-alpha-1");

    // Client 2 subscribes to /beta with id "sub-beta-1"
    assert(send_tcp_line(client2, "{\"op\":\"subscribe\",\"topic\":\"/beta\",\"id\":\"sub-beta-1\"}"));
    std::string stat2 = read_tcp_line(client2);
    assert(!stat2.empty());
    JsonValue parsed_stat2 = JsonValue::parse(stat2);
    assert(parsed_stat2["op"].as_string() == "status");
    assert(parsed_stat2["level"].as_string() == "info");
    assert(parsed_stat2["id"].as_string() == "sub-beta-1");

    // A client must register each topic before it can publish on that topic.
    assert(send_tcp_line(client3, "{\"op\":\"advertise\",\"topic\":\"/alpha\",\"type\":\"example/Message\"}"));
    assert(send_tcp_line(client3, "{\"op\":\"advertise\",\"topic\":\"/beta\",\"type\":\"example/Message\"}"));

    // Client 3 publishes to /alpha
    assert(send_tcp_line(client3, "{\"op\":\"publish\",\"topic\":\"/alpha\",\"msg\":{\"hello\":\"world\"}}"));

    // Associated with Step 3: Client 1 should receive publication
    std::string line1 = read_tcp_line(client1);
    assert(!line1.empty() && "Client 1 must receive published message");
    JsonValue parsed1 = JsonValue::parse(line1);
    assert(parsed1["op"].as_string() == "publish");
    assert(parsed1["topic"].as_string() == "/alpha");
    assert(parsed1["msg"]["hello"].as_string() == "world");

    // Associated with Step 4: Client 2 should NOT receive publication on /alpha (topic isolation)
    std::string line2 = read_tcp_line(client2, 200);
    assert(line2.empty() && "Client 2 must not receive messages from unrelated topic /alpha");

    // Now Client 3 publishes to /beta
    assert(send_tcp_line(client3, "{\"op\":\"publish\",\"topic\":\"/beta\",\"msg\":{\"count\":123}}"));
    std::string line2_beta = read_tcp_line(client2);
    assert(!line2_beta.empty());
    JsonValue parsed2 = JsonValue::parse(line2_beta);
    assert(parsed2["op"].as_string() == "publish");
    assert(parsed2["topic"].as_string() == "/beta");
    assert(parsed2["msg"]["count"].as_int() == 123);

    // Withdrawing a connection's advertisement immediately prevents it from
    // publishing to that topic, while leaving its other advertisements intact.
    assert(send_tcp_line(client3, "{\"op\":\"unadvertise\",\"topic\":\"/alpha\"}"));
    assert(send_tcp_line(client3, "{\"op\":\"publish\",\"topic\":\"/alpha\",\"msg\":{\"hello\":\"after-unadvertise\"}}"));
    std::string after_unadvertise = read_tcp_line(client1, 200);
    assert(after_unadvertise.empty() && "Unadvertised clients must not publish to that topic");

    close(client1);
    close(client2);
    close(client3);
    server.stop();
    std::cout << "  -> PASSED" << std::endl;
}

/*
1. Test calling services over external TCP/JSON gateway.
2. Verify service response is routed to the calling client.
3. Verify request/response IDs are correlated correctly.
4. Verify calling from independent clients preserves isolation.
*/
void test_tcp_gateway_service_call() {
    std::cout << "[TEST] TCP Gateway Service Calls & ID Correlation..." << std::endl;
    Middleware mw;
    int test_port = 9095;
    RosbridgeServer server(mw, "127.0.0.1", test_port);
    assert(server.start());
    std::this_thread::sleep_for(std::chrono::milliseconds(50));

    // Register service on middleware
    mw.advertise_service("/square", [](const JsonValue& args) -> std::pair<bool, JsonValue> {
        double val = args["number"].as_number();
        JsonValue res = JsonValue::make_object();
        res["result"] = val * val;
        return {true, res};
    });

    int client1 = connect_tcp("127.0.0.1", test_port);
    int client2 = connect_tcp("127.0.0.1", test_port);
    assert(client1 >= 0 && client2 >= 0);

    // Client 1 calls service with id "call_A"
    assert(send_tcp_line(client1, "{\"op\":\"call_service\",\"service\":\"/square\",\"args\":{\"number\":6},\"id\":\"call_A\"}"));

    // Client 2 calls service with id "call_B"
    assert(send_tcp_line(client2, "{\"op\":\"call_service\",\"service\":\"/square\",\"args\":{\"number\":9},\"id\":\"call_B\"}"));

    // Read response for Client 1
    std::string resp1_str = read_tcp_line(client1);
    assert(!resp1_str.empty());
    JsonValue resp1 = JsonValue::parse(resp1_str);
    assert(resp1["op"].as_string() == "service_response");
    assert(resp1["service"].as_string() == "/square");
    assert(resp1["id"].as_string() == "call_A");
    assert(resp1["result"].as_bool() == true);
    assert(resp1["values"]["result"].as_int() == 36);

    // Read response for Client 2
    std::string resp2_str = read_tcp_line(client2);
    assert(!resp2_str.empty());
    JsonValue resp2 = JsonValue::parse(resp2_str);
    assert(resp2["op"].as_string() == "service_response");
    assert(resp2["service"].as_string() == "/square");
    assert(resp2["id"].as_string() == "call_B");
    assert(resp2["result"].as_bool() == true);
    assert(resp2["values"]["result"].as_int() == 81);

    close(client1);
    close(client2);
    server.stop();
    std::cout << "  -> PASSED" << std::endl;
}

/*
1. Test external service providers registered via advertise_service over TCP.
2. Verify call_service is forwarded to provider and response routed back to caller.
*/
void test_tcp_external_service_provider() {
    std::cout << "[TEST] External Service Provider over TCP..." << std::endl;
    Middleware mw;
    int test_port = 9095;
    RosbridgeServer server(mw, "127.0.0.1", test_port);
    assert(server.start());
    std::this_thread::sleep_for(std::chrono::milliseconds(50));

    // Provider client connects and advertises /echo
    int provider = connect_tcp("127.0.0.1", test_port);
    assert(provider >= 0);
    assert(send_tcp_line(provider, "{\"op\":\"advertise_service\",\"service\":\"/echo\",\"type\":\"std_srvs/SetBool\",\"id\":\"adv-1\"}"));

    // Provider receives status confirming registration
    std::string adv_stat = read_tcp_line(provider);
    assert(!adv_stat.empty());
    JsonValue p_stat = JsonValue::parse(adv_stat);
    assert(p_stat["op"].as_string() == "status");
    assert(p_stat["id"].as_string() == "adv-1");

    // Caller client connects and calls /echo
    int caller = connect_tcp("127.0.0.1", test_port);
    assert(caller >= 0);
    assert(send_tcp_line(caller, "{\"op\":\"call_service\",\"service\":\"/echo\",\"args\":{\"echo\":\"hello\"},\"id\":\"call-17\"}"));

    // Provider receives forwarded call_service
    std::string fwd_str = read_tcp_line(provider);
    assert(!fwd_str.empty());
    JsonValue fwd = JsonValue::parse(fwd_str);
    assert(fwd["op"].as_string() == "call_service");
    assert(fwd["service"].as_string() == "/echo");
    assert(fwd["args"]["echo"].as_string() == "hello");
    std::string prov_id = fwd["id"].as_string();

    // Provider responds using received ID
    std::string prov_resp = "{\"op\":\"service_response\",\"service\":\"/echo\",\"id\":\"" + prov_id + "\",\"values\":{\"echo\":\"hello\"},\"result\":true,\"status\":\"\"}";
    assert(send_tcp_line(provider, prov_resp));

    // Caller receives response with original ID "call-17"
    std::string caller_resp_str = read_tcp_line(caller);
    assert(!caller_resp_str.empty());
    JsonValue caller_resp = JsonValue::parse(caller_resp_str);
    assert(caller_resp["op"].as_string() == "service_response");
    assert(caller_resp["id"].as_string() == "call-17");
    assert(caller_resp["result"].as_bool() == true);
    assert(caller_resp["values"]["echo"].as_string() == "hello");

    close(provider);
    close(caller);
    server.stop();
    std::cout << "  -> PASSED" << std::endl;
}

/*
1. Test that client disconnection cleans up associated subscriptions.
2. Ensure system remains functional for subsequent clients after disconnections.
*/
void test_tcp_client_disconnect_cleanup() {
    std::cout << "[TEST] TCP Client Disconnect & Cleanup..." << std::endl;
    Middleware mw;
    int test_port = 9095;
    RosbridgeServer server(mw, "127.0.0.1", test_port);
    assert(server.start());
    std::this_thread::sleep_for(std::chrono::milliseconds(50));

    // Client connects and subscribes
    int client = connect_tcp("127.0.0.1", test_port);
    assert(client >= 0);
    assert(send_tcp_line(client, "{\"op\":\"subscribe\",\"topic\":\"/cleanup_topic\",\"id\":\"c1\"}"));
    std::string stat = read_tcp_line(client);
    assert(!stat.empty());

    // Verify subscriber is registered on middleware
    assert(mw.has_subscribers("/cleanup_topic"));

    // Disconnect client abruptly
    close(client);
    std::this_thread::sleep_for(std::chrono::milliseconds(100));

    // Stale subscription must be purged
    assert(!mw.has_subscribers("/cleanup_topic") && "Subscriptions must be purged when client disconnects");

    server.stop();
    std::cout << "  -> PASSED" << std::endl;
}

/*
1. Execute all unit and integration test suites for the ROS-like middleware.
*/
int main() {
    std::cout << "========================================" << std::endl;
    std::cout << "Running ROS-Like Middleware Acceptance Tests" << std::endl;
    std::cout << "========================================" << std::endl;

    test_in_process_topics();
    test_in_process_services();
    test_tcp_gateway_pubsub();
    test_tcp_gateway_service_call();
    test_tcp_external_service_provider();
    test_tcp_client_disconnect_cleanup();

    std::cout << "========================================" << std::endl;
    std::cout << "All Middleware tests PASSED successfully!" << std::endl;
    std::cout << "========================================" << std::endl;
    return 0;
}
