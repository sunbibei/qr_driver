/*
 * label.h
 *
 *  Created on: Aug 28, 2017
 *      Author: silence
 */

#ifndef INCLUDE_SYSTEM_LABEL_LABEL_H_
#define INCLUDE_SYSTEM_LABEL_LABEL_H_

#include <string>
#include <map>

#include <boost/shared_ptr.hpp>
#include <system/utils/utf.h>


namespace middleware {

class Label {
  friend class AutoInstanceor;
public:
  typedef boost::shared_ptr<Label> LabelPtr;
  const static MiiString null;

  Label(ConstRef<MiiString> l, ConstRef<MiiString> p = Label::null);
  Label(ConstRef<MiiString> l, const Label& p);
  Label(ConstRef<MiiString> l, LabelPtr p);
  Label(ConstRef<MiiString> l, Label* p);

  virtual ~Label();

  ConstRef<MiiString> getLabel() { return label_; }
  ConstRef<MiiString> getLabel() const { return label_; }

  static MiiString make_label  (ConstRef<MiiString>, ConstRef<MiiString>);
  static MiiString parent_label(ConstRef<MiiString>);
  static void      split_label (MiiString, MiiString&, MiiString&);

  template<class _Hardware>
  static _Hardware* getHardwareByName(const MiiString&);

  // For Debug
  static void printfEveryInstance() {
    LOG_WARNING << "Label's table, address " << &s_label_table_
        << ", size " << s_label_table_.size();
    LOG_WARNING << "============================================";
    int count = 0;
    for (auto l : s_label_table_) {
      LOG_WARNING << count++ << ": " << l.second->getLabel();
    }
    LOG_WARNING << "============================================";
  }

protected:
  MiiString label_;

protected:
  /**
   * @brief Initialization will be completed here.
   *        The default implementation is null, All of subclass
   *        judge whether or not it need to override.
   *        Note that you must be fill the label_ before call this method.
   */
  virtual bool init();

  static MiiMap<MiiString, LabelPtr>& label_table() { return s_label_table_; }

  /**
   * TODO
   * 不应该出现的函数，并且还使用了友元类的方式才解决了注册问题。
   * 妥协方案，下一步应该处理这个问题
   */
  static void registerClass(LabelPtr& l_sp, ConstRef<MiiString> __l) {
    label_table().insert(std::make_pair(l_sp->getLabel(), l_sp));
  }

private:
  static MiiMap<MiiString, LabelPtr> s_label_table_;
};



///////////////////////////////////////////////////////////////////////////////
////////////        The implementation of template methods         ////////////
///////////////////////////////////////////////////////////////////////////////
template<class _Hardware>
_Hardware* Label::getHardwareByName(ConstRef<MiiString> l) {
  auto hw = label_table().find(l);
  if ((label_table().end() == hw)
      || (nullptr == boost::dynamic_pointer_cast<_Hardware>(hw->second))) {
    return nullptr;
  }

  return static_cast<_Hardware*>(hw->second.get());
}

} /* namespace middleware */

#endif /* INCLUDE_SYSTEM_LABEL_LABEL_H_ */
