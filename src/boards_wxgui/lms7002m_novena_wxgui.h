/**
@file	lms7002m_novena_wxgui.h
@author Lime Microsystems (www.limemicro.com)
@brief	User interface for controls on Novena board
*/

#ifndef LMS7002M_NOVENA_WXGUI_H
#define LMS7002M_NOVENA_WXGUI_H

#include <wx/wx.h>
#include "lime/LimeSuite.h"


class LMS7002M_Novena_wxgui : public wxPanel
{
public:
    LMS7002M_Novena_wxgui(wxWindow* parent, wxWindowID id = wxID_ANY, const wxString &title = wxEmptyString, const wxPoint& pos = wxDefaultPosition, const wxSize& size = wxDefaultSize, long styles = 0);
    void Initialize(lms_device_t* serPort, const size_t devIndex = 0);
    virtual ~LMS7002M_Novena_wxgui();
    virtual void UpdatePanel();

    wxCheckBox *lms_reset, *lms_rxen, *lms_txen, *lms_gpio2, *lms_gpio1, *lms_gpio0;
protected:
    void ParameterChangeHandler(wxCommandEvent& event);
    void OnReadAll(wxCommandEvent& event);
    void OnWriteAll(wxCommandEvent &event);
    lms_device_t* lmsControl;
    int m_rficSpiAddr;
    DECLARE_EVENT_TABLE()
};

#endif //LMS7002M_NOVENA_H
