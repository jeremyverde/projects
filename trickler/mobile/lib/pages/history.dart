/// Copyright (c) Ammolytics and contributors. All rights reserved.
/// Released under the MIT license. See LICENSE file in the project root for details.

import 'package:flutter/material.dart';
import 'package:flutter_redux/flutter_redux.dart';
import '../models/index.dart';
import '../widgets/header.dart';

/// HistoryPage is responsible for passing a title to _HistoryPageState.

class HistoryPage extends StatefulWidget {
  HistoryPage({ Key key }) : super(key: key);

  final String title = 'Measurement History';

  @override
  _HistoryPageState createState() => _HistoryPageState();
}

/// _HistoryPageState is meant to be a place that displays all measurments in the measurementHistory.
/// However, right now it just displays the current targetWieght.

class _HistoryPageState extends State<HistoryPage> {
  @override
  Widget build(BuildContext context) {
    return StoreConnector<AppState, AppState>(
      converter: (store) => store.state,
      builder: (context, state) {
        return Scaffold(
          appBar: PreferredSize(
            preferredSize: Size.fromHeight(60.0),
              child: Header(
              key: Key('Header'),
              title: widget.title,
            ),
          ),
          body: Center(
            child: Column(
              mainAxisAlignment: MainAxisAlignment.center,
              children: <Widget>[
                Text('Target Weight ${state.currentMeasurement.targetWeight}'),
              ],
            ),
          ),
        );
      },
    );
  }
}