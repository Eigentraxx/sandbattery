exports.solarMower = functions.database.ref('data/{pushId}')
  .onWrite(async (change, context) => {
    // Only edit data when it is first created.
    if (change.before.exists()) {
      return null;
    }
    // Exit when the data is deleted.
    if (!change.after.exists()) {
      return null;
    }
    const original = change.after.val();


    db.ref('batterymower/authTime').once('value', (a) => {
      var b = a.val();
      console.log('ts', b);
      let currentDate = Date.now();
      var hours = Math.abs(currentDate - b) / 36e5;
      // tokens expire in 24 hours. Get new token. skip the request.
      if (hours > 22) {
        getAndSaveNewAuthToken();
      } else {
        db.ref('batterymower/auth').once('value', (a) => {
          var b = a.val();
          getAndSaveMowerData(b, 'authtime');
        });
      }
    });
  });
