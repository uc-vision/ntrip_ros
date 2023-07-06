from http.client import HTTPResponse, BadStatusLine, RemoteDisconnected, LineTooLong

_MAXLINE = 65536


class NTRIPResponse(HTTPResponse):

    def _read_status(self):
        line = str(self.fp.readline(_MAXLINE + 1), "iso-8859-1")
        if len(line) > _MAXLINE:
            raise LineTooLong("status line")
        if self.debuglevel > 0:
            print("reply:", repr(line))
        if not line:
            # Presumably, the server closed the connection before
            # sending a valid response.
            raise RemoteDisconnected("Remote end closed connection without"
                                     " response")
        try:
            version, status, reason = line.split(None, 2)
        except ValueError:
            try:
                version, status = line.split(None, 1)
                reason = ""
            except ValueError:
                # empty version will cause next test to fail.
                version = ""
        if version.startswith("ICY"):
            version = "HTTP/1.0"
        if not version.startswith("HTTP/"):
            self._close_conn()
            raise BadStatusLine(line)

        # The status code is a three-digit number
        try:
            status = int(status)
            if status < 100 or status > 999:
                raise BadStatusLine(line)
        except ValueError:
            raise BadStatusLine(line)
        return version, status, reason