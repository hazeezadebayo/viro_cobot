import http.server
import socketserver
import sys

class QuietHandler(http.server.SimpleHTTPRequestHandler):
    # Suppress normal logging if desired, or keep default
    pass

class QuietServer(socketserver.ThreadingTCPServer):
    def handle_error(self, request, client_address):
        # Ignore benign network errors like a user interrupting a download
        import traceback
        exc_type, exc_value, exc_traceback = sys.exc_info()
        if exc_type in (BrokenPipeError, ConnectionResetError):
            return  # Completely silent for these
            
        # For other errors, print the traceback normally
        print('-'*40, file=sys.stderr)
        print('Exception occurred during processing of request from', client_address, file=sys.stderr)
        traceback.print_exc()
        print('-'*40, file=sys.stderr)

if __name__ == '__main__':
    port = int(sys.argv[1]) if len(sys.argv) > 1 else 8080
    socketserver.TCPServer.allow_reuse_address = True
    with QuietServer(("0.0.0.0", port), QuietHandler) as httpd:
        print(f"Serving HTTP on 0.0.0.0 port {port} (Filtering BrokenPipeErrors)")
        try:
            httpd.serve_forever()
        except KeyboardInterrupt:
            print("\nShutting down server.")
