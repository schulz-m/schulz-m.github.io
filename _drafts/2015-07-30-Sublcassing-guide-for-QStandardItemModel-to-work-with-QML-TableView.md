---
layout: post
title: "Sublcassing guide for QStandardItemModel to work with QML TableView"
tags:
- Qt
- QML
- QtQuick2
---

*Note:* QML and QtQuick2 are equivalent in this article.

`QStandardItemModel` is a general purpose Qt C++ class that implements the `QAbstractItemModel` interface such that QML or QtWidgets get notified of data changes. It extends this last class by taking care of the storage of the data. More info [here](http://doc.qt.io/qt-5/qstandarditemmodel.html).

The QML TableView is a visual elemnent that renders a table from an underlying model. [Here](http://doc.qt.io/qt-5/qml-qtquick-controls-tableview.html#details) is an example in pure QML.

For a variety of reasons such as increasing data complexity or separation of data and the QML view, the data will be moved to a C++ layer of the Qt application.

// todo more

# Subclassing `QStandardItemModel`

For brevity the `QStandardItemModel` or another `QAbstractItemModel` derived class will be referred to as simply "model". The QML TableView will be referred to as simply "view".

The view assumes that rows to display are a list in the model. This means that the elements are located from `QModelIntex(0,0)` to `QModelIntex(0, nrOfRows - 1)` . When adding or removing elements one has to think in terms of rows.

The columns are assumed to be the roles of the elements in this list, one can think of them as properties of a row. To define the roles one has to reimplement the `QAbstractItemModel::roleNames()` function.

For a practical example lets say that the columns in this model should be: name, age, city. Then our code would look like the following.

```cpp
#include <QStandardItemModel>

class VehiclePersitentInformationTable : public QStandardItemModel {
  Q_OBJECT
public:
  VehiclePersitentInformationTable(QObject *parent){
    Q_UNUSED(parent)
	// Build QHash for role names
	_roleEnumToString[static_cast< int >(Roles::NAME)] = "name";
	_roleEnumToString[static_cast< int >(Roles::CITY)] = "city";
	_roleEnumToString[static_cast< int >(Roles::AGE)]  = "age";
  }

  enum Roles{
    NAME  = Qt::UserRole,
    CITY,
    AGE
  };

  QHash<int, QByteArray> roleNames() const {
    return _roleEnumToString;
  }

private:
  QHash<int, QByteArray> _roleEnumToString;
};

```

When we need to refer to one of these columns/properties we will have to use the `enum Roles` on the C++ side and the string we registered in the constructor on the QML side.

# Usage example


'This article is incomplete'