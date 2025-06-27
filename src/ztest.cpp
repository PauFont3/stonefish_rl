#include <zmq.hpp>
#include <iostream>
#include <string>

int main() {

    zmq::context_t context(1);
    zmq::socket_t socket(context, zmq::socket_type::rep);  // REP = servidor
    socket.bind("tcp://*:5555");
    std::cout << "[C++] Servidor REP escoltant a 5555...\n";

    while(true) {
        zmq::message_t request;

        // Esperar a rebre el missatge
        auto result = socket.recv(request, zmq::recv_flags::none);

        // Comprovar si s'ha rebut algun missatge
        if(!result.has_value()) {
            std::cerr << "[C++] No s'ha rebut cap missatge." << std::endl;
            continue;
        }

        // Convertir el missatge (que esta en un buffer) a string i mostrar-lo
        std::string msg = request.to_string();
        std::cout << "[C++] He rebut: (" << result.value() << " bytes): " << msg << std::endl;

        // Preparar la resposta que s'enviara al client (cap a pyhton en aquest cas)
        std::string reply = " Hola desde C++";

        // Enviar la resposta
        socket.send(zmq::buffer(reply), zmq::send_flags::none);
    }
    return 0;
}
