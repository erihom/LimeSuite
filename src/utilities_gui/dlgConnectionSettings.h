#ifndef __dlgConnectionSettings__
#define __dlgConnectionSettings__

/**
@file
Subclass of dlgConnectionSettings, which is generated by wxFormBuilder.
*/

#include "utilities_gui.h"
#include <ConnectionRegistry.h>

//// end generated include

class IConnection;

/** Implementing dlgConnectionSettings */
class dlgConnectionSettings : public dlgConnectionSettings_view
{
	protected:
		// Handlers for dlgConnectionSettings events.
		void GetDeviceList( wxInitDialogEvent& event );
		void OnConnect( wxCommandEvent& event );
		void OnCancel( wxCommandEvent& event );
		void OnDisconnect( wxCommandEvent& event );
	public:
		/** Constructor */
		dlgConnectionSettings( wxWindow* parent );
	//// end generated class members
	protected:
		IConnection *lms7Conn;
		int lmsOpenedIndex;
		IConnection *streamBrdConn;
		int streamOpenedIndex;
		std::vector<ConnectionHandle> cachedHandles;
};

#endif // __dlgConnectionSettings__
